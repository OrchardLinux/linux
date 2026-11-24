// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright (c) 2023 Imagination Technologies Ltd. */

#include "pvr_apple.h"
#include "pvr_device.h"
#include "pvr_rogue_fwif_stream.h"
#include "pvr_stream.h"

#include <linux/align.h>
#include <linux/dev_printk.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <uapi/drm/pvr_drm.h>

static __always_inline bool
stream_def_is_supported(struct pvr_device *pvr_dev, const struct pvr_stream_def *stream_def)
{
	if (stream_def->feature == PVR_FEATURE_NONE)
		return true;

	if (!(stream_def->feature & PVR_FEATURE_NOT) &&
	    pvr_device_has_feature(pvr_dev, stream_def->feature)) {
		return true;
	}

	if ((stream_def->feature & PVR_FEATURE_NOT) &&
	    !pvr_device_has_feature(pvr_dev, stream_def->feature & ~PVR_FEATURE_NOT)) {
		return true;
	}

	return false;
}

static int
pvr_stream_get_data(u8 *stream, u32 *stream_offset, u32 stream_size, u32 data_size, u32 align_size,
		    void *dest)
{
	*stream_offset = ALIGN(*stream_offset, align_size);

	if ((*stream_offset + data_size) > stream_size)
		return -EINVAL;

	memcpy(dest, stream + *stream_offset, data_size);

	(*stream_offset) += data_size;

	return 0;
}

/**
 * pvr_stream_process_1() - Process a single stream and fill destination structure
 * @pvr_dev: Device pointer.
 * @type: Command stream type.
 * @ext_type: Extension header type, or -1 for the main stream.
 * @stream_def: Stream definition.
 * @nr_entries: Number of entries in &stream_def.
 * @stream: Pointer to stream.
 * @stream_offset: Starting offset within stream.
 * @stream_size: Size of input stream, in bytes.
 * @dest: Pointer to destination structure.
 * @dest_size: Size of destination structure.
 * @stream_offset_out: Pointer to variable to write updated stream offset to. May be NULL.
 *
 * Returns:
 *  * 0 on success, or
 *  * -%EINVAL on malformed stream.
 */
static int
pvr_stream_process_1(struct pvr_device *pvr_dev, enum pvr_stream_type type, int ext_type,
		     const struct pvr_stream_def *stream_def, u32 nr_entries, u8 *stream,
		     u32 stream_offset, u32 stream_size,
		     u8 *dest, u32 dest_size, u32 *stream_offset_out)
{
	int err = 0;

	for (u32 i = 0; i < nr_entries; i++) {
		u32 data_size = 0;

		if (stream_def[i].offset >= dest_size) {
			dev_warn_ratelimited(from_pvr_device(pvr_dev)->dev,
					     "stream rejected: destination type=%u ext_type=%d entry=%u offset=%u dest_size=%u\n",
					     type, ext_type, i, stream_def[i].offset, dest_size);
			err = -EINVAL;
			break;
		}

		if (!stream_def_is_supported(pvr_dev, &stream_def[i]))
			continue;

		switch (stream_def[i].size) {
		case PVR_STREAM_SIZE_8:
			data_size = sizeof(u8);
			err = pvr_stream_get_data(stream, &stream_offset, stream_size, sizeof(u8),
						  sizeof(u8), dest + stream_def[i].offset);
			break;

		case PVR_STREAM_SIZE_16:
			data_size = sizeof(u16);
			err = pvr_stream_get_data(stream, &stream_offset, stream_size, sizeof(u16),
						  sizeof(u16), dest + stream_def[i].offset);
			break;

		case PVR_STREAM_SIZE_32:
			data_size = sizeof(u32);
			err = pvr_stream_get_data(stream, &stream_offset, stream_size, sizeof(u32),
						  sizeof(u32), dest + stream_def[i].offset);
			break;

		case PVR_STREAM_SIZE_64:
			data_size = sizeof(u64);
			err = pvr_stream_get_data(stream, &stream_offset, stream_size, sizeof(u64),
						  sizeof(u64), dest + stream_def[i].offset);
			break;

		case PVR_STREAM_SIZE_ARRAY:
			data_size = stream_def[i].array_size;
			err = pvr_stream_get_data(stream, &stream_offset, stream_size,
						  stream_def[i].array_size, sizeof(u64),
						  dest + stream_def[i].offset);
			break;
		}
		if (err) {
			dev_warn_ratelimited(from_pvr_device(pvr_dev)->dev,
					     "stream rejected: short data type=%u ext_type=%d entry=%u offset=%u size=%u stream_size=%u\n",
					     type, ext_type, i, stream_offset, data_size, stream_size);
			return err;
		}
	}

	if (stream_offset_out)
		*stream_offset_out = stream_offset;

	return err;
}

static int
pvr_stream_process_ext_stream(struct pvr_device *pvr_dev,
			      const struct pvr_stream_cmd_defs *cmd_defs, void *ext_stream,
			      u32 stream_offset, u32 ext_stream_size, void *dest)
{
	u32 musthave_masks[PVR_STREAM_EXTHDR_TYPE_MAX];
	u32 ext_header;
	int err = 0;

	/* Copy "must have" mask from device. We clear this as we process the stream. */
	memcpy(musthave_masks, pvr_dev->stream_musthave_quirks[cmd_defs->type],
	       sizeof(musthave_masks));

	do {
		const struct pvr_stream_ext_header *header;
		u32 type;
		u32 data;

		err = pvr_stream_get_data(ext_stream, &stream_offset, ext_stream_size, sizeof(u32),
					  sizeof(ext_header), &ext_header);
		if (err) {
			dev_warn_ratelimited(from_pvr_device(pvr_dev)->dev,
					     "stream rejected: short extension header type=%u ext_type=-1 entry=-1 offset=%u size=%zu stream_size=%u\n",
					     cmd_defs->type, stream_offset, sizeof(ext_header), ext_stream_size);
			return err;
		}

		type = (ext_header & PVR_STREAM_EXTHDR_TYPE_MASK) >> PVR_STREAM_EXTHDR_TYPE_SHIFT;
		data = ext_header & PVR_STREAM_EXTHDR_DATA_MASK;

		if (type >= cmd_defs->ext_nr_headers) {
			dev_warn_ratelimited(from_pvr_device(pvr_dev)->dev,
					     "stream rejected: extension type=%u ext_type=%u ext_nr_headers=%u\n",
					     cmd_defs->type, type, cmd_defs->ext_nr_headers);
			return -EINVAL;
		}

		header = &cmd_defs->ext_headers[type];
		if (data & ~header->valid_mask) {
			dev_warn_ratelimited(from_pvr_device(pvr_dev)->dev,
					     "stream rejected: extension bits type=%u ext_type=%u data=0x%08x valid_mask=0x%08x\n",
					     cmd_defs->type, type, data, header->valid_mask);
			return -EINVAL;
		}

		musthave_masks[type] &= ~data;

		for (u32 i = 0; i < header->ext_streams_num; i++) {
			const struct pvr_stream_ext_def *ext_def = &header->ext_streams[i];

			if (!(ext_header & ext_def->header_mask))
				continue;

			/* Quirk 0: the extension is not tied to a hardware quirk. */
			if (ext_def->quirk && !pvr_device_has_uapi_quirk(pvr_dev, ext_def->quirk)) {
				dev_warn_ratelimited(from_pvr_device(pvr_dev)->dev,
						     "stream rejected: unsupported quirk type=%u ext_type=%u entry=%u quirk=%u data=0x%08x mask=0x%08x\n",
						     cmd_defs->type, type, i, ext_def->quirk, data, ext_def->header_mask);
				return -EINVAL;
			}

			err = pvr_stream_process_1(pvr_dev, cmd_defs->type, type,
						   ext_def->stream, ext_def->stream_len,
						   ext_stream, stream_offset,
						   ext_stream_size, dest,
						   cmd_defs->dest_size, &stream_offset);
			if (err)
				return err;
		}
	} while (ext_header & PVR_STREAM_EXTHDR_CONTINUATION);

	/*
	 * Verify that "must have" mask is now zero. If it isn't then one of the "must have" quirks
	 * for this command was not present.
	 */
	for (u32 i = 0; i < cmd_defs->ext_nr_headers; i++) {
		if (musthave_masks[i]) {
			dev_warn_ratelimited(from_pvr_device(pvr_dev)->dev,
					     "stream rejected: missing must-have type=%u ext_type=%u leftover_mask=0x%08x\n",
					     cmd_defs->type, i, musthave_masks[i]);
			return -EINVAL;
		}
	}

	return 0;
}

/**
 * pvr_stream_process() - Build FW structure from stream
 * @pvr_dev: Device pointer.
 * @cmd_defs: Stream definition.
 * @stream: Pointer to command stream.
 * @stream_size: Size of command stream, in bytes.
 * @dest_out: Pointer to destination buffer.
 *
 * Caller is responsible for freeing the output structure.
 *
 * Returns:
 *  * 0 on success,
 *  * -%ENOMEM on out of memory, or
 *  * -%EINVAL on malformed stream.
 */
int
pvr_stream_process(struct pvr_device *pvr_dev, const struct pvr_stream_cmd_defs *cmd_defs,
		   void *stream, u32 stream_size, void *dest_out)
{
	u32 stream_offset = 0;
	u32 main_stream_len;
	u32 padding;
	int err;

	if (!stream || !stream_size) {
		dev_warn_ratelimited(from_pvr_device(pvr_dev)->dev,
				     "stream rejected: empty input type=%u stream_present=%u stream_size=%u\n",
				     cmd_defs->type, !!stream, stream_size);
		return -EINVAL;
	}

	err = pvr_stream_get_data(stream, &stream_offset, stream_size, sizeof(u32),
				  sizeof(u32), &main_stream_len);
	if (err) {
		dev_warn_ratelimited(from_pvr_device(pvr_dev)->dev,
				     "stream rejected: short main_stream_len type=%u ext_type=-1 entry=-1 offset=%u size=%zu stream_size=%u\n",
				     cmd_defs->type, stream_offset, sizeof(u32), stream_size);
		return err;
	}

	/*
	 * u32 after stream length is padding to ensure u64 alignment, but may be used for expansion
	 * in the future. Verify it's zero.
	 */
	err = pvr_stream_get_data(stream, &stream_offset, stream_size, sizeof(u32),
				  sizeof(u32), &padding);
	if (err) {
		dev_warn_ratelimited(from_pvr_device(pvr_dev)->dev,
				     "stream rejected: short padding type=%u ext_type=-1 entry=-1 offset=%u size=%zu stream_size=%u\n",
				     cmd_defs->type, stream_offset, sizeof(u32), stream_size);
		return err;
	}

	if (main_stream_len < stream_offset || main_stream_len > stream_size || padding) {
		dev_warn_ratelimited(from_pvr_device(pvr_dev)->dev,
				     "stream rejected: main header type=%u main_stream_len=%u stream_size=%u padding=0x%08x\n",
				     cmd_defs->type, main_stream_len, stream_size, padding);
		return -EINVAL;
	}

	err = pvr_stream_process_1(pvr_dev, cmd_defs->type, -1,
				   cmd_defs->main_stream, cmd_defs->main_stream_len,
				   stream, stream_offset, main_stream_len, dest_out,
				   cmd_defs->dest_size, &stream_offset);
	if (err)
		return err;

	if (stream_offset < stream_size) {
		err = pvr_stream_process_ext_stream(pvr_dev, cmd_defs, stream, stream_offset,
						    stream_size, dest_out);
		if (err)
			return err;
	} else {
		/*
		 * If we don't have an extension stream then there must not be any "must have"
		 * quirks for this command.
		 */
		for (u32 i = 0; i < cmd_defs->ext_nr_headers; i++) {
			if (pvr_dev->stream_musthave_quirks[cmd_defs->type][i]) {
				dev_warn_ratelimited(from_pvr_device(pvr_dev)->dev,
						     "stream rejected: no extension must-have type=%u ext_type=%u leftover_mask=0x%08x\n",
						     cmd_defs->type, i,
						     pvr_dev->stream_musthave_quirks[cmd_defs->type][i]);
				return -EINVAL;
			}
		}
	}

	return 0;
}

/**
 * pvr_stream_create_musthave_masks() - Create "must have" masks for streams based on current device
 *                                      quirks
 * @pvr_dev: Device pointer.
 */
void
pvr_stream_create_musthave_masks(struct pvr_device *pvr_dev)
{
	memset(pvr_dev->stream_musthave_quirks, 0, sizeof(pvr_dev->stream_musthave_quirks));

	if (pvr_device_has_uapi_quirk(pvr_dev, 47217))
		pvr_dev->stream_musthave_quirks[PVR_STREAM_TYPE_FRAG][0] |=
			PVR_STREAM_EXTHDR_FRAG0_BRN47217;

	if (pvr_device_is_apple(pvr_dev)) {
		u32 ext_type, mask;

		mask = pvr_apple_stream_musthave(pvr_dev, PVR_STREAM_TYPE_GEOM, &ext_type);
		pvr_dev->stream_musthave_quirks[PVR_STREAM_TYPE_GEOM][ext_type] |= mask;
		mask = pvr_apple_stream_musthave(pvr_dev, PVR_STREAM_TYPE_FRAG, &ext_type);
		pvr_dev->stream_musthave_quirks[PVR_STREAM_TYPE_FRAG][ext_type] |= mask;
	}

	if (pvr_device_has_uapi_quirk(pvr_dev, 49927)) {
		pvr_dev->stream_musthave_quirks[PVR_STREAM_TYPE_GEOM][0] |=
			PVR_STREAM_EXTHDR_GEOM0_BRN49927;
		pvr_dev->stream_musthave_quirks[PVR_STREAM_TYPE_FRAG][0] |=
			PVR_STREAM_EXTHDR_FRAG0_BRN49927;
		pvr_dev->stream_musthave_quirks[PVR_STREAM_TYPE_COMPUTE][0] |=
			PVR_STREAM_EXTHDR_COMPUTE0_BRN49927;
	}
}

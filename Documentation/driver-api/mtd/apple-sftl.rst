.. SPDX-License-Identifier: GPL-2.0-only

Apple PPN VFL/SwissPPNFTL
========================

The ``apple-sftl`` driver recovers the logical medium above Apple's PPN flash
mapping. H2FMI supplies complete 16 KiB pages and their 64 metadata bytes
through an aggregate provider. Context reads use the package's separate SLC
address view. The controller serializes each complete transaction.

Supported format
----------------

This implementation supports the measured 64 GB Hynix PPN profile with two
buses and eight CAUs per bus. Each CAU has 2,088 blocks of 128 pages. Profile
selection uses discovered geometry. Other layouts return ``-EOPNOTSUPP``.

Recovery selects four agreeing VFL checkpoint copies and checks each
replicated lane table. A VFL superblock has an 8,192-unit address stride;
units are 4 KiB. Blocks with fewer active lanes have an unused address gap
which the reader rejects.

SFTL subtype-4 context fragments describe the saved logical ranges. Recovery
validates the complete context sequence, then applies later data and trim
records in sequence order. Completed blocks carry summaries at their ends.
Open blocks are scanned completely because an interrupted stripe may leave an
erased lane before another completed lane. Summary allocation spans also
preserve the next sequence floor when it exceeds the last visible record.

The map has 15,625,016 entries, including sixteen internal keys. The block
device exposes 15,625,000 entries, or 64,000,000,000 bytes, with 512-byte
sector reads and optional read-modify-write sector updates. Linux's disk name
is ``applesftlN``. LwVM and filesystem interpretation happen above this
logical provider. Encrypted partition contents remain ciphertext.

Failure and lifetime rules
--------------------------

Malformed context copies and incomplete journal sequences prevent disk
registration. A refresh status is reported as an error. A failed runtime read stops subsequent I/O from this map instance.

H2FMI unregisters the child before releasing its controller state. The block
frontend freezes and drains requests during removal. Its request buffer is
allocated before the disk becomes visible, while blocking requests serialize
access to that buffer and the provider. Request processing uses a NOIO
allocation scope so map and provider buffers cannot trigger recursive
writeback while the queue lock is held.

Append writes
-------------

The aggregate provider must accept a complete 16
KiB page with its 64 metadata bytes, wait for completion, and verify exact
readback before reporting success. The FTL publishes the new mapping after
that success. A short logical write repeats its final 4 KiB entry to fill the
physical program unit; each copy carries its own sequence.

The allocator reserves every saved SFTL context block, all validated VFL
bootstrap member blocks, every block referenced by the recovered map, and the
boot and package tail blocks. It selects a block whose head is erased, then
checks every physical page and metadata byte on every active lane before
programming any page. An occupied candidate is skipped permanently within the
instance. A failed scan stops I/O. Allocation scans have a 500-second limit;
the block queue allows 600 seconds for the scan and synchronous programs.

Data records use native type-1 metadata. Once a block's data region is full,
the driver writes eight summary pages containing native type-1c run entries.
The old Apple checkpoint and the shared map remain intact. Fresh recovery
replays completed data records from an open block, or uses a complete summary.
An interrupted summary can be ignored only when its metadata identifies the
reserved summary tail and the entire preceding data prefix validates in
physical and sequence order. Torn or ambiguous metadata stops recovery.

The recovered allocation floor can exceed the last visible journal record.
This implementation retains its contiguous journal replay rule, so a mismatch
keeps the disk read-only and returns ``-EOPNOTSUPP`` before any program.
Native forward-gap handling requires a separate recovery change with format-
specific fixtures. A program or map-publication error stops subsequent I/O; a
fresh loader must determine which complete records reached the medium.

The block frontend preserves untouched sectors in partial 4 KiB updates and
groups up to four logical pages per program. Every acknowledged write has
completed readback. Flush and FUA use that same completion rule. There is no
erase, discard, checkpoint update, or garbage collection. Allocation returns
``-ENOSPC`` when verified blank blocks run out, and reads remain available.
Writes are only enabled when the provider can program pages and the replayed
journal ends at the allocation floor.

Tests
-----

``tools/testing/kunit/configs/apple_sftl.config`` enables the synthetic medium
tests; a synthetic provider stands in for the controller. The fixtures
generate VFL replicas and lane tables, plus an SFTL checkpoint followed by
completed and open data blocks. Tests cover trim ordering and allocation floors. They also program synthetic pages and load a
fresh map after open and closed blocks, interrupted summaries, and uncertain
programs. Fixtures also cover complete candidate scans, VFL member
reservations, space exhaustion, sequence-floor refusal and the final 48-bit
sequence. Corrupt replicas and sequence gaps must return an error with no disk
or usable map. Test inputs contain no captured device bytes.

The format rules were ported from Orchard's P105 Python reference
(``ftl_map.py``, ``ftl_load.py``, ``ftl_log.py`` and ``ftl_append.py``), which
was checked against native iOS logical reads. A kernel object compile or
synthetic test result does not establish physical-controller timing or qualify
a new device layout.

/* SPDX-License-Identifier: GPL-2.0 */
#ifndef DECOMPRESS_PROGRESS_H
#define DECOMPRESS_PROGRESS_H

/*
 * decompress_progress(pos) - progress hook for in-memory decompressors
 *
 * Called from the inner loops of the slow decompressors (LZMA, XZ) with a
 * position that advances by exactly one per call. Boot decompressors whose
 * board must keep doing something while a multi-second decompression runs
 * with interrupts off (e.g. feeding an external watchdog) can define it
 * before including the lib/ decompressor sources. Everywhere else it is a
 * no-op.
 */
#ifndef decompress_progress
#define decompress_progress(pos) do { } while (0)
#endif

#endif /* DECOMPRESS_PROGRESS_H */

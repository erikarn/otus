/*
 * Copyright (c) 2015 Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef __ATHP_HAL_SWAP_H__
#define __ATHP_HAL_SWAP_H__

#define ATH10K_SWAP_CODE_SEG_BIN_LEN_MAX	(512 * 1024)
#define ATH10K_SWAP_CODE_SEG_MAGIC_BYTES_SZ	12
#define ATH10K_SWAP_CODE_SEG_NUM_MAX		16
/* Currently only one swap segment is supported */
#define ATH10K_SWAP_CODE_SEG_NUM_SUPPORTED	1

struct ath10k_swap_code_seg_tlv {
	__le32 address;
	__le32 length;
	u8 data[0];
} __packed;

struct ath10k_swap_code_seg_tail {
	u8 magic_signature[ATH10K_SWAP_CODE_SEG_MAGIC_BYTES_SZ];
	__le32 bmi_write_addr;
} __packed;

union ath10k_swap_code_seg_item {
	struct ath10k_swap_code_seg_tlv tlv;
	struct ath10k_swap_code_seg_tail tail;
} __packed;

enum ath10k_swap_code_seg_bin_type {
	 ATH10K_SWAP_CODE_SEG_BIN_TYPE_OTP,
	 ATH10K_SWAP_CODE_SEG_BIN_TYPE_FW,
	 ATH10K_SWAP_CODE_SEG_BIN_TYPE_UTF,
};

struct ath10k_swap_code_seg_hw_info {
	/* Swap binary image size */
	__le32 swap_size;
	__le32 num_segs;

	/* Swap data size */
	__le32 size;
	__le32 size_log2;
	__le32 bus_addr[ATH10K_SWAP_CODE_SEG_NUM_MAX];
	__le64 reserved[ATH10K_SWAP_CODE_SEG_NUM_MAX];
} __packed;

#endif

/*
 * Copyright (c) 2017-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef NVGPU_POSIX_VM__REMAP_H
#define NVGPU_POSIX_VM_REMAP_H

#include <nvgpu/types.h>

struct nvgpu_vm_remap_os_buffer {
	struct nvgpu_os_buffer os_buf;
	struct nvgpu_mapped_buf_priv os_priv;

	struct nvgpu_sgt *nv_sgt;
	u64 aperture;
};

u64 nvgpu_vm_remap_get_handle(struct nvgpu_vm_remap_os_buffer *remap_os_buf);
int nvgpu_vm_remap_os_buf_get(struct vm_gk20a *vm, u32 mem_handle,
			struct nvgpu_vm_remap_os_buffer *remap_os_buf);
void nvgpu_vm_remap_os_buf_put(struct vm_gk20a *vm,
			struct nvgpu_vm_remap_os_buffer *remap_os_buf);

#endif /* NVGPU_POSIX_VM_REMAP_H */

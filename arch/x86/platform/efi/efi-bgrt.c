/*
 * Copyright 2012 Intel Corporation
 * Author: Josh Triplett <josh@joshtriplett.org>
 *
 * Based on the bgrt driver:
 * Copyright 2012 Red Hat, Inc <mjg@redhat.com>
 * Author: Matthew Garrett
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/acpi.h>
#include <linux/efi.h>
#include <linux/efi-bgrt.h>

struct acpi_table_bgrt *bgrt_tab;
void *__initdata bgrt_image;
size_t __initdata bgrt_image_size;

struct bmp_header {
	u16 id;
	u32 size;
} __packed;

void __init efi_bgrt_init(void)
{
	acpi_status status;
	void *image;
	struct bmp_header bmp_header;

	if (acpi_disabled)
		return;

	status = acpi_get_table("BGRT", 0,
	                        (struct acpi_table_header **)&bgrt_tab);
	if (ACPI_FAILURE(status))
		return;

	if (bgrt_tab->header.length < sizeof(*bgrt_tab))
		return;
	if (bgrt_tab->version != 1)
		return;
	if (bgrt_tab->image_type != 0 || !bgrt_tab->image_address)
		return;

	image = memremap(bgrt_tab->image_address, sizeof(bmp_header), MEMREMAP_WB);
	if (!image) {
		pr_err("Ignoring BGRT: failed to map image header memory\n");
		return;
	}

	memcpy(&bmp_header, image, sizeof(bmp_header));
	memunmap(image);
	bgrt_image_size = bmp_header.size;

	bgrt_image = kmalloc(bgrt_image_size, GFP_KERNEL);
	if (!bgrt_image)
		return;

	image = memremap(bgrt_tab->image_address, bmp_header.size, MEMREMAP_WB);
	if (!image) {
		pr_err("Ignoring BGRT: failed to map image memory\n");
		kfree(bgrt_image);
		bgrt_image = NULL;
		return;
	}

	memcpy(bgrt_image, image, bgrt_image_size);
	memunmap(image);
}

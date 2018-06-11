/*
 * Copyright (c) 2018 Loongson Technology Co., Ltd.
 * Authors:
 *	Zhu Chen <zhuchen@loongson.cn>
 *	Fang Yaling <fangyaling@loongson.cn>
 *	Zhang Dandan <zhangdandan@loongson.cn>
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include "loongson_drv.h"

void * loongson_vbios_test(void){
	struct loongson_vbios *vbios;
	struct loongson_vbios_crtc * crtc_vbios[2];
	struct loongson_vbios_connector *connector_vbios[2];
	struct loongson_vbios_phy *phy_vbios[2];

	vbios = kzalloc(120*1024,GFP_KERNEL);

	/*Build loongson_vbios struct*/
	vbios->version_major = 0;
	vbios->version_minor = 0;
	vbios->crtc_num = 2;
	vbios->crtc_offset = sizeof(struct loongson_vbios);
	vbios->connector_num = 2;
	vbios->connector_offset = sizeof(struct loongson_vbios) + 2 * sizeof(struct loongson_vbios_crtc);
	vbios->phy_num = 2;
	vbios->phy_offset =
		sizeof(struct loongson_vbios) + 2 * sizeof(struct loongson_vbios_crtc) + 2 * sizeof(struct loongson_vbios_connector);


	/*Build loongson_vbios_crtc struct*/
	crtc_vbios[0] = (struct loongson_vbios_crtc *)(vbios + vbios->crtc_offset);
	crtc_vbios[1] = (struct loongson_vbios_crtc *)(vbios + vbios->crtc_offset + sizeof(struct loongson_vbios_crtc));

	crtc_vbios[0]->next_crtc_offset = sizeof(struct loongson_vbios) + sizeof(struct loongson_vbios_crtc);
	crtc_vbios[0]->crtc_id = 0;
	crtc_vbios[0]->crtc_max_weight = 2048;
	crtc_vbios[0]->crtc_max_height = 2048;
	crtc_vbios[0]->connecotor_id = 0;
	crtc_vbios[0]->phy_num = 1;
	crtc_vbios[0]->phy_id[0] = 0;

	crtc_vbios[1]->next_crtc_offset = NULL;
	crtc_vbios[1]->crtc_id = 1;
	crtc_vbios[1]->crtc_max_weight = 2048;
	crtc_vbios[1]->crtc_max_height = 2048;
	crtc_vbios[1]->connecotor_id = 1;
	crtc_vbios[1]->phy_num = 1;
	crtc_vbios[1]->phy_id[0] = 1;

	/*Build loongson_vbios_connector struct*/
	connector_vbios[0] = (struct loongson_vbios_connector *)(vbios + vbios->connector_offset);
	connector_vbios[1] = (struct loongson_vbios_connector *)(vbios + vbios->connector_offset + sizeof(struct loongson_vbios_connector));

	/*Build loongson_vbios_phy struct*/
	phy_vbios[0] = (struct loongson_vbios_phy *)(vbios + vbios->phy_offset);
	phy_vbios[1] = (struct loongson_vbios_phy *)(vbios + vbios->phy_offset + sizeof(struct loongson_vbios_phy));

	return (void *)vbios;
}

int loongson_vbios_init(struct loongson_drm_device *ldev){
	struct loongson_vbios *vbios;
	int i;
	/*get a test vbios,just for test*/
	ldev->vbios = (struct loongson_vbios *)loongson_vbios_test();

	vbios = ldev->vbios;
	if(vbios == NULL)
		return -1;

	/*get crtc struct points*/
	ldev->crtc_vbios[0] = (struct loongson_vbios_crtc *)(vbios + vbios->crtc_offset);
	if(vbios->crtc_num > 1)
	{
		for(i = 1;i < vbios->crtc_num; i++){
		ldev->crtc_vbios[i] = (struct loongson_vbios_crtc *)(vbios + ldev->crtc_vbios[i - 1]->next_crtc_offset);
		}
	}

	/*get connector struct points*/
	ldev->connector_vbios [0] = (struct loongson_vbios_connector *)(vbios + vbios->connector_offset);
	if(vbios->connector_num > 1){
		for(i = 1;i < vbios->connector_num; i++){
		ldev->connector_vbios[i] = (struct loongson_vbios_connector *)(vbios + ldev->connector_vbios[i - 1]->next_connector_offset);
		}
	}

	/*get phy struct points*/
	ldev->phy_vbios[0] = (struct loongson_vbios_phy *)(vbios + vbios->phy_offset);
	if(vbios->phy_num > 1){
		for(i = 1;i < vbios->phy_num; i++){
		ldev->phy_vbios[1] = (struct loongson_vbios_phy *)(vbios + ldev->phy_vbios[0]->next_phy_offset);
		}
	}

	return 0;
}

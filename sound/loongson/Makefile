ls-snd-hda-objs := ls_hda.o

ls-snd-hda-codec-y := hda_codec.o
ls-snd-hda-codec-$(CONFIG_PROC_FS) += hda_proc.o

ls-snd-hda-codec-realtek-objs :=	patch_realtek.o

# common driver
obj-$(CONFIG_LS_SND_HDA) := ls-snd-hda-codec.o

obj-$(CONFIG_LS_SND_HDA) += ls-snd-hda-codec-realtek.o

# this must be the last entry after codec drivers;
obj-$(CONFIG_LS_SND_HDA) += ls-snd-hda.o

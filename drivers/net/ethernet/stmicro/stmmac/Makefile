obj-$(CONFIG_STMMAC_ETH) += stmmac.o
stmmac-$(CONFIG_STMMAC_PLATFORM) += stmmac_platform.o
stmmac-$(CONFIG_STMMAC_PCI) += stmmac_pci.o
stmmac-$(CONFIG_DWMAC_LPC18XX) += dwmac-lpc18xx.o
stmmac-$(CONFIG_DWMAC_MESON) += dwmac-meson.o
stmmac-$(CONFIG_DWMAC_SUNXI) += dwmac-sunxi.o
stmmac-$(CONFIG_DWMAC_STI) += dwmac-sti.o
stmmac-$(CONFIG_DWMAC_SOCFPGA) += dwmac-socfpga.o
stmmac-objs:= stmmac_main.o stmmac_ethtool.o stmmac_mdio.o ring_mode.o	\
	      chain_mode.o dwmac_lib.o dwmac1000_core.o  dwmac1000_dma.o \
	      dwmac100_core.o dwmac100_dma.o enh_desc.o  norm_desc.o \
	      mmc_core.o stmmac_hwtstamp.o stmmac_ptp.o $(stmmac-y)

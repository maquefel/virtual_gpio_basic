#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("pci:v00001AF4d00001110sv00000010sd00000108bc*sc*i*");
MODULE_ALIAS("pci:v00001AF4d00001110sv00000010sd00000208bc*sc*i*");
MODULE_ALIAS("pci:v00001AF4d00001110sv00000010sd00000308bc*sc*i*");

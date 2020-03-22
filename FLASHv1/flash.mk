ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_FLASH TRUE,$(HALCONF)),)
PLATFORMSRC += FLASHv1/hal_flash_lld.c \
               FLASHv1/hal_flash.c
endif
else
PLATFORMSRC += FLASHv1/hal_flash_lld.c \
               FLASHv1/hal_flash.c
endif

PLATFORMINC += FLASHv1

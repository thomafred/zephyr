if (CONFIG_HAS_SILABS_RAIL)
  zephyr_include_directories(silabs_rail/platform/radio/rail_lib/emlib/inc)
  zephyr_include_directories(silabs_rail/platform/radio/rail_lib/chip/efr32)
  zephyr_include_directories(silabs_rail/platform/radio/rail_lib/common)
  zephyr_include_directories(silabs_rail/platform/radio/rail_lib/protocol/ieee802154)
  zephyr_include_directories(silabs_rail/platform/radio/rail_lib/plugin/pa-conversions)

  add_library(silabs_rail STATIC IMPORTED GLOBAL)
  set_target_properties(silabs_rail PROPERTIES IMPORTED_LOCATION
    ${ZEPHYR_BASE}/ext/lib/radio/silabs_rail/platform/radio/rail_lib/autogen/librail_release/librail_efr32xg12_gcc_release.a)
  zephyr_append_cmake_library(silabs_rail)
endif() 

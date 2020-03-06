set(COMPONENT_ADD_INCLUDEDIRS include)

# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES httpc streams)
set(COMPONENT_PRIV_REQUIRES console)

set(COMPONENT_SRCS src/abstract_rb.c src/abstract_rb_utils.c src/basic_rb.c src/diag_cli.c src/esp_audio_mem.c src/linked_list.c src/m3u8_parser.c src/pls_parser.c src/scli.c src/special_rb.c src/utils.c)

register_component()

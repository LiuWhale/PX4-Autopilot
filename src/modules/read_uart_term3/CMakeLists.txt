set(MODULE_CFLAGS)
px4_add_module(
        MODULE modules__read_uart_term3
        MAIN read_uart_term3
    COMPILE_FLAGS
        -Os
    SRCS
                read_uart_term3.c
    DEPENDS
        #platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 


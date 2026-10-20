# Input tracker for 3DoF XR glasses (VITURE, XREAL One-series, RayNeo).
#
# Talks to the glasses directly: HID for VITURE and RayNeo, TCP over the
# glasses' USB Ethernet link for XREAL. No vendor SDK or driver is needed to
# build or run. Windows x64 only.
if(WIN32)
    otr_module(tracker-xr-glasses)
    # setupapi: device enumeration; hid: product strings, report sizes;
    # ws2_32: Winsock for the XREAL network transport.
    target_link_libraries(${self} PRIVATE setupapi hid ws2_32)
endif()

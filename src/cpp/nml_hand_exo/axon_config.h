#pragma once

// Opt-in composite USB build for the patched SciFi server. Default firmware
// retains its dual CDC layout. Set for ALL translation units when compiling.
#ifndef EXO_AXON_USB
#define EXO_AXON_USB 0
#endif

#if EXO_AXON_USB && !defined(ARDUINO_OpenRB)
#error "EXO_AXON_USB requires OpenRB-150"
#endif

// Hardware type, NOT the runtime Synapse peripheral_id assigned by SciFi.
// Must match firmware/axon-exo/manifest.json and the plugin registration.
#define EXO_AXON_TYPE_ID 0xF002u

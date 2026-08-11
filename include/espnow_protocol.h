#pragma once

// Shared ESP-NOW wire structs for the tractor <-> seeder link. Both boards
// include this same header so the struct layout used by esp_now_send/
// memcpy can never drift out of sync between the two firmwares.

// Seeder -> Tractor
typedef struct struct_seeder {
    int turbineRPM;
    int WOMRPM;
    bool mechanismTurning;
    bool tramlineActive;
} struct_seeder;

// Tractor -> Seeder
typedef struct struct_tractor {
    bool tramlineActive;
} struct_tractor;

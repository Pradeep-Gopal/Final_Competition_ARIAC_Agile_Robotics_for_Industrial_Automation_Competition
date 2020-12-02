#include "utils.h"

/**
 * Hash-map to get different model heights for different parts.
 * They were modified because each part sinks into the surface a bit,
 * in varying amounts.
 */
std::unordered_map<std::string, double> model_height = { {
    "piston_rod_part_red", 0.0065 },
    { "piston_rod_part_green", 0.0065 }, { "piston_rod_part_blue", 0.0065 }, {
        "pulley_part_red", 0.07 }, { "pulley_part_green", 0.07 }, {
        "pulley_part_blue", 0.07 }, { "gear_part_red", 0.012 }, {
        "gear_part_green", 0.012 }, { "gear_part_blue", 0.012 }, {
        "gasket_part_red", 0.02 }, { "gasket_part_green", 0.02 }, {
        "gasket_part_blue", 0.02 }, { "disk_part_red", 0.023 }, {
        "disk_part_green", 0.02 }, { "disk_part_blue", 0.023 } };

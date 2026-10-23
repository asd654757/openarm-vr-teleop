// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

namespace {

volatile std::sig_atomic_t keep_running = 1;

void signal_handler(int /*signum*/) { keep_running = 0; }

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct Config {
    std::string interface = "can0";
    std::string output = "openarm_pinn_raw.csv";
    std::vector<uint32_t> send_ids = {1};
    std::vector<openarm::damiao_motor::MotorType> motor_types = {
        openarm::damiao_motor::MotorType::DM8009};
    uint32_t recv_offset = 0x10;
    bool use_fd = true;
    bool request_state = true;
    double rate_hz = 50.0;
    double duration_s = 30.0;
    int chunk = 0;
    std::string joint_prefix = "openarm_joint";
    int flush_every = 25;
    int param_response_ms = 8;
};

std::vector<std::string> split_csv(const std::string& text) {
    std::vector<std::string> out;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) out.push_back(item);
    }
    return out;
}

std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

uint32_t parse_u32(const std::string& text) {
    return static_cast<uint32_t>(std::stoul(text, nullptr, 0));
}

openarm::damiao_motor::MotorType parse_motor_type(const std::string& text) {
    using openarm::damiao_motor::MotorType;
    const std::string name = lower(text);
    if (name == "dm3507") return MotorType::DM3507;
    if (name == "dm4310") return MotorType::DM4310;
    if (name == "dm4310_48v" || name == "dm4310-48v") return MotorType::DM4310_48V;
    if (name == "dm4340") return MotorType::DM4340;
    if (name == "dm4340_48v" || name == "dm4340-48v") return MotorType::DM4340_48V;
    if (name == "dm6006") return MotorType::DM6006;
    if (name == "dm8006") return MotorType::DM8006;
    if (name == "dm8009") return MotorType::DM8009;
    if (name == "dm10010l") return MotorType::DM10010L;
    if (name == "dm10010") return MotorType::DM10010;
    if (name == "dmh3510") return MotorType::DMH3510;
    if (name == "dmh6215") return MotorType::DMH6215;
    if (name == "dmg6220") return MotorType::DMG6220;
    throw std::invalid_argument("unknown motor type: " + text);
}

std::string motor_type_name(openarm::damiao_motor::MotorType type) {
    using openarm::damiao_motor::MotorType;
    switch (type) {
        case MotorType::DM3507:
            return "DM3507";
        case MotorType::DM4310:
            return "DM4310";
        case MotorType::DM4310_48V:
            return "DM4310_48V";
        case MotorType::DM4340:
            return "DM4340";
        case MotorType::DM4340_48V:
            return "DM4340_48V";
        case MotorType::DM6006:
            return "DM6006";
        case MotorType::DM8006:
            return "DM8006";
        case MotorType::DM8009:
            return "DM8009";
        case MotorType::DM10010L:
            return "DM10010L";
        case MotorType::DM10010:
            return "DM10010";
        case MotorType::DMH3510:
            return "DMH3510";
        case MotorType::DMH6215:
            return "DMH6215";
        case MotorType::DMG6220:
            return "DMG6220";
        default:
            return "UNKNOWN";
    }
}

double param_or_nan(const openarm::damiao_motor::Motor& motor, openarm::damiao_motor::RID rid) {
    const double value = motor.get_param(static_cast<int>(rid));
    if (!std::isfinite(value) || value == -1.0) return kNaN;
    return value;
}

void print_usage(const char* program) {
    std::cout
        << "Usage: " << program << " [options]\n"
        << "\n"
        << "Read-only OpenArm PINN/friction raw data logger. It never enables motors,\n"
        << "disables motors, writes registers, sets zero, or sends motion commands.\n"
        << "\n"
        << "Options:\n"
        << "  --interface IFACE        SocketCAN interface (default: can0)\n"
        << "  --output PATH            CSV path (default: openarm_pinn_raw.csv)\n"
        << "  --ids LIST               Send CAN IDs, comma-separated (default: 1)\n"
        << "  --types LIST             Motor types, comma-separated (default: DM8009)\n"
        << "  --recv-offset N          Receive ID offset (default: 0x10)\n"
        << "  --rate HZ                Target logging rate (default: 50)\n"
        << "  --duration SEC           Duration in seconds, <=0 runs until Ctrl+C (default: 30)\n"
        << "  --chunk N                Chunk/trajectory segment ID (default: 0)\n"
        << "  --joint-prefix PREFIX    Joint name prefix (default: openarm_joint)\n"
        << "  --classic-can            Use classic CAN instead of CAN-FD\n"
        << "  --passive                Do not send refresh requests; only parse received frames\n"
        << "  --flush-every N          Flush every N written samples (default: 25)\n"
        << "  --param-response-ms N    Wait after each param query (default: 8)\n"
        << "  -h, --help               Show this message\n";
}

Config parse_args(int argc, char* argv[]) {
    Config cfg;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument(name + " requires a value");
            return argv[++i];
        };

        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        } else if (arg == "--interface") {
            cfg.interface = require_value(arg);
        } else if (arg == "--output") {
            cfg.output = require_value(arg);
        } else if (arg == "--ids") {
            cfg.send_ids.clear();
            for (const auto& item : split_csv(require_value(arg))) cfg.send_ids.push_back(parse_u32(item));
        } else if (arg == "--types") {
            cfg.motor_types.clear();
            for (const auto& item : split_csv(require_value(arg))) {
                cfg.motor_types.push_back(parse_motor_type(item));
            }
        } else if (arg == "--recv-offset") {
            cfg.recv_offset = parse_u32(require_value(arg));
        } else if (arg == "--rate") {
            cfg.rate_hz = std::stod(require_value(arg));
        } else if (arg == "--duration") {
            cfg.duration_s = std::stod(require_value(arg));
        } else if (arg == "--chunk") {
            cfg.chunk = std::stoi(require_value(arg));
        } else if (arg == "--joint-prefix") {
            cfg.joint_prefix = require_value(arg);
        } else if (arg == "--classic-can") {
            cfg.use_fd = false;
        } else if (arg == "--passive") {
            cfg.request_state = false;
        } else if (arg == "--flush-every") {
            cfg.flush_every = std::max(1, std::stoi(require_value(arg)));
        } else if (arg == "--param-response-ms") {
            cfg.param_response_ms = std::max(0, std::stoi(require_value(arg)));
        } else {
            throw std::invalid_argument("unknown option: " + arg);
        }
    }

    if (cfg.send_ids.empty()) throw std::invalid_argument("at least one motor ID is required");
    if (cfg.motor_types.empty()) throw std::invalid_argument("at least one motor type is required");
    if (cfg.motor_types.size() == 1 && cfg.send_ids.size() > 1) {
        cfg.motor_types.assign(cfg.send_ids.size(), cfg.motor_types.front());
    }
    if (cfg.motor_types.size() != cfg.send_ids.size()) {
        throw std::invalid_argument("--types must have one entry or match --ids length");
    }
    if (cfg.rate_hz <= 0.0) throw std::invalid_argument("--rate must be positive");
    return cfg;
}

void query_param(openarm::can::socket::OpenArm& openarm, openarm::damiao_motor::RID rid,
                 int response_ms) {
    openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);
    openarm.query_param_all(static_cast<int>(rid));
    if (response_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(response_ms));
    openarm.recv_all(2000);
}

void warn_missing_params(const std::vector<openarm::damiao_motor::Motor>& motors,
                         const std::vector<uint32_t>& send_ids,
                         const std::vector<openarm::damiao_motor::RID>& rids,
                         const std::string& context) {
    for (size_t i = 0; i < motors.size(); ++i) {
        for (const auto rid : rids) {
            if (!std::isfinite(param_or_nan(motors[i], rid))) {
                std::cerr << "WARNING: missing " << context << " RID "
                          << static_cast<int>(rid) << " for motor ID 0x" << std::hex
                          << send_ids[i] << std::dec << "\n";
            }
        }
    }
}

void write_csv_header(std::ofstream& out) {
    out << "time,chunk,joint_name,joint_index,motor_id,motor_type,"
           "gear_ratio,tmax_nm,kt_value,"
           "feedback_pos,feedback_vel,feedback_torque,"
           "t_mos,t_rotor,"
           "p_m,xout,"
           "command_mode,command_pos,command_vel,command_torque,command_kp,command_kd\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    try {
        Config cfg = parse_args(argc, argv);

        std::vector<uint32_t> recv_ids;
        recv_ids.reserve(cfg.send_ids.size());
        for (const auto id : cfg.send_ids) recv_ids.push_back(id + cfg.recv_offset);

        std::ofstream out(cfg.output);
        if (!out) throw std::runtime_error("failed to open output CSV: " + cfg.output);
        out << std::setprecision(12);
        write_csv_header(out);

        openarm::can::socket::OpenArm openarm(cfg.interface, cfg.use_fd);
        openarm.init_arm_motors(cfg.motor_types, cfg.send_ids, recv_ids);

        std::cout << "OpenArm PINN logger on " << cfg.interface << " (" << (cfg.use_fd ? "CAN-FD" : "classic CAN")
                  << "), motors=" << cfg.send_ids.size() << ", rate=" << cfg.rate_hz
                  << " Hz, request_state=" << (cfg.request_state ? "true" : "false") << "\n";
        std::cout << "Safety: not enabling motors, not writing registers, not commanding motion.\n";

        for (const auto rid : {openarm::damiao_motor::RID::Gr,
                               openarm::damiao_motor::RID::TMAX,
                               openarm::damiao_motor::RID::KT_Value}) {
            query_param(openarm, rid, cfg.param_response_ms);
        }
        warn_missing_params(openarm.get_arm().get_motors(), cfg.send_ids,
                            {openarm::damiao_motor::RID::Gr,
                             openarm::damiao_motor::RID::TMAX,
                             openarm::damiao_motor::RID::KT_Value},
                            "static parameter");

        const auto start = std::chrono::steady_clock::now();
        auto next_wakeup = start;
        const auto cycle_duration =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / cfg.rate_hz));

        uint64_t rows_written = 0;
        uint64_t loop_count = 0;

        while (keep_running) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed = std::chrono::duration<double>(now - start).count();
            if (cfg.duration_s > 0.0 && elapsed >= cfg.duration_s) break;

            if (cfg.request_state) {
                openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
                openarm.refresh_all();
                openarm.recv_all(2000);
            } else {
                openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
                openarm.recv_all(2000);
            }

            query_param(openarm, openarm::damiao_motor::RID::p_m, cfg.param_response_ms);
            query_param(openarm, openarm::damiao_motor::RID::xout, cfg.param_response_ms);

            const auto motors = openarm.get_arm().get_motors();
            for (size_t i = 0; i < motors.size(); ++i) {
                const auto& motor = motors[i];
                const std::string joint_name = cfg.joint_prefix + std::to_string(i + 1);
                const double gear_ratio = param_or_nan(motor, openarm::damiao_motor::RID::Gr);
                const double tmax = param_or_nan(motor, openarm::damiao_motor::RID::TMAX);
                const double kt = param_or_nan(motor, openarm::damiao_motor::RID::KT_Value);
                const double p_m = param_or_nan(motor, openarm::damiao_motor::RID::p_m);
                const double xout = param_or_nan(motor, openarm::damiao_motor::RID::xout);

                out << elapsed << ',' << cfg.chunk << ',' << joint_name << ',' << i << ','
                    << cfg.send_ids[i] << ',' << motor_type_name(cfg.motor_types[i]) << ','
                    << gear_ratio << ',' << tmax << ',' << kt << ','
                    << motor.get_position() << ',' << motor.get_velocity() << ','
                    << motor.get_torque() << ',' << motor.get_state_tmos() << ','
                    << motor.get_state_trotor() << ',' << p_m << ',' << xout << ','
                    << "none," << kNaN << ',' << kNaN << ',' << kNaN << ',' << kNaN << ','
                    << kNaN << '\n';
                ++rows_written;
            }

            if (rows_written % static_cast<uint64_t>(cfg.flush_every) == 0) out.flush();
            ++loop_count;

            next_wakeup += cycle_duration;
            const auto before_sleep = std::chrono::steady_clock::now();
            if (next_wakeup > before_sleep) std::this_thread::sleep_until(next_wakeup);
        }

        out.flush();
        const double total_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        const double sample_hz = total_s > 0.0 ? static_cast<double>(loop_count) / total_s : 0.0;
        std::cout << "Wrote " << rows_written << " rows to " << cfg.output << "\n";
        std::cout << "Actual logging frequency: " << std::fixed << std::setprecision(2)
                  << sample_hz << " Hz per motor\n";

        const auto final_motors = openarm.get_arm().get_motors();
        warn_missing_params(final_motors, cfg.send_ids,
                            {openarm::damiao_motor::RID::p_m,
                             openarm::damiao_motor::RID::xout},
                            "dynamic parameter");
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}

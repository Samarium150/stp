/**
 * Copyright (c) 2025. Samarium
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef PROCESS_INFO_H_
#define PROCESS_INFO_H_

#ifdef _WIN32
// clang-format off
#include <windows.h>
#include <psapi.h>
// clang-format on
#elif defined(__APPLE__)
#include <mach/mach.h>
#include <sys/resource.h>
#else  // Linux
#include <sys/resource.h>

#include <fstream>
#endif

struct ProcessUsage {
    double user_cpu_time_seconds{};
    double system_cpu_time_seconds{};
    double total_cpu_time_seconds{};
    uint64_t resident_memory_bytes{};
    uint64_t resident_memory_peak_bytes{};
    uint64_t virtual_memory_bytes{};
};

inline ProcessUsage GetProcessUsage() {
    ProcessUsage usage{};
#ifdef _WIN32
    FILETIME creation_time, exit_time, kernel_time, user_time;
    if (GetProcessTimes(GetCurrentProcess(), &creation_time, &exit_time, &kernel_time,
                        &user_time)) {
        auto FileTimeToSeconds = [](const FILETIME &ft) -> double {
            ULARGE_INTEGER uli;
            uli.LowPart = ft.dwLowDateTime;
            uli.HighPart = ft.dwHighDateTime;
            return static_cast<double>(uli.QuadPart) / 1e7;
        };
        usage.user_cpu_time_seconds = FileTimeToSeconds(user_time);
        usage.system_cpu_time_seconds = FileTimeToSeconds(kernel_time);
        usage.total_cpu_time_seconds = usage.user_cpu_time_seconds + usage.system_cpu_time_seconds;
    }
    PROCESS_MEMORY_COUNTERS_EX pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS *>(&pmc),
                             sizeof(pmc))) {
        usage.resident_memory_bytes = pmc.WorkingSetSize;
        usage.resident_memory_peak_bytes = pmc.PeakWorkingSetSize;
        usage.virtual_memory_bytes = pmc.PrivateUsage;
    }
#elif defined(__APPLE__)
    rusage ru{};
    if (getrusage(RUSAGE_SELF, &ru) == 0) {
        usage.user_cpu_time_seconds =
            static_cast<double>(ru.ru_utime.tv_sec) + ru.ru_utime.tv_usec / 1e6;
        usage.system_cpu_time_seconds =
            static_cast<double>(ru.ru_utime.tv_sec) + ru.ru_stime.tv_usec / 1e6;
        usage.total_cpu_time_seconds = usage.user_cpu_time_seconds + usage.system_cpu_time_seconds;
    }
    mach_task_basic_info_data_t info;
    mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO, reinterpret_cast<task_info_t>(&info),
                  &count) == KERN_SUCCESS) {
        usage.resident_memory_bytes = info.resident_size;
        usage.resident_memory_peak_bytes = info.resident_size_max;
        usage.virtual_memory_bytes = info.virtual_size;
    }
#else
    rusage ru{};
    if (getrusage(RUSAGE_SELF, &ru) == 0) {
        usage.user_cpu_time_seconds = ru.ru_utime.tv_sec + ru.ru_utime.tv_usec / 1e6;
        usage.system_cpu_time_seconds = ru.ru_stime.tv_sec + ru.ru_stime.tv_usec / 1e6;
        usage.total_cpu_time_seconds = usage.user_cpu_time_seconds + usage.system_cpu_time_seconds;
    }
    std::ifstream file("/proc/self/status");
    std::string line;
    while (std::getline(file, line)) {
        if (line.compare(0, 7, "VmSize:") == 0) {
            std::istringstream iss(line);
            std::string label, unit;
            std::uint64_t value;
            if (iss >> label >> value >> unit) {
                usage.virtual_memory_bytes = value * 1024;
            }
        } else if (line.compare(0, 6, "VmHWM:") == 0) {
            std::istringstream iss(line);
            std::string label, unit;
            std::uint64_t value;
            if (iss >> label >> value >> unit) {
                usage.resident_memory_peak_bytes = value * 1024;
            }
        } else if (line.compare(0, 6, "VmRSS:") == 0) {
            std::istringstream iss(line);
            std::string label, unit;
            std::uint64_t value;
            if (iss >> label >> value >> unit) {
                usage.resident_memory_bytes = value * 1024;
            }
        }
    }
#endif
    return usage;
}
#endif  // PROCESS_INFO_H_

/* mbed Microcontroller Library
 * Copyright (c) 2020 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef BLE_PM_SENSE_SERVICE_H
#define BLE_PM_SENSE_SERVICE_H

#include "ble/BLE.h"

#if BLE_FEATURE_GATT_SERVER

/** This is a custom Particulate Matter Sensing which exposes PM counts and the Sample Update frequency.
 *
 * The PM Count characteristic added is read and notify only. Do no construct this class.
 * Use the static method add_service to add the chosen Device Information Service characteristics to the server.
 *
 * You can read the specification of the service on the bluetooth website, currently at:
 * https://www.bluetooth.com/specifications/specs/
 * Otherwise search the website for "Device Information Service".
 */
class PMSenseService {
public:
    /** Adds device-specific information into the BLE stack. This must only be called once.
     *
     * @param[in] ble A reference to a BLE object for the underlying controller.
     * @param[in] Custom Gatt Service UUID.
     * @param[in] Custom Characteristic UUID for PM Counts.
     * @param[in] Custom Characteristic UUID for PM Intervals.
     *
     * @note Do not call more than once. Calling this multiple times will create multiple
     * instances of the service which is against the spec.
     */
    static ble_error_t add_service(
        BLE &ble,
        const char *gattservice_uuid    = nullptr,
        const char *pmcountchar_uuid    = nullptr,
        const char *pmintervalchar_uuid = nullptr
    );

private:
    PMSenseService() = delete;
    ~PMSenseService() = delete;
};

#endif // BLE_FEATURE_GATT_SERVER

#endif /* #ifndef BLE_DEVICE_INFORMATION_SERVICE_H */

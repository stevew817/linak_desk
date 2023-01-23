/*
 *
 *    Copyright (c) 2020 Project CHIP Authors
 *    Copyright (c) 2019 Google LLC.
 *    All rights reserved.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include <AppTask.h>

#include "AppConfig.h"
#include "init_efrPlatform.h"
#include "sl_simple_button_instances.h"
#include "sl_system_kernel.h"
#include <matter_config.h>
#include <DeviceInfoProviderImpl.h>
#include <app/server/Server.h>
#include <app/clusters/window-covering-server/window-covering-server.h>
#include <credentials/DeviceAttestationCredsProvider.h>
#ifdef EFR32_ATTESTATION_CREDENTIALS
#include <examples/platform/silabs/SilabsDeviceAttestationCreds.h>
#else
#include <credentials/examples/DeviceAttestationCredsExample.h>
#endif

// The name must not be longer than 13 characters
#define BLE_DEV_NAME "Linak-Desk"
using namespace ::chip;
using namespace ::chip::Inet;
using namespace ::chip::DeviceLayer;
using namespace ::chip::Credentials;

#define UNUSED_PARAMETER(a) (a = a)

volatile int apperror_cnt;
static chip::DeviceLayer::DeviceInfoProviderImpl gExampleDeviceInfoProvider;

#include "WindowCoveringImpl.h"
static WindowCoveringImpl gWindowCovering;

// ================================================================================
// Main Code
// ================================================================================
int main(void)
{
    init_efrPlatform();
    if (EFR32MatterConfig::InitMatter(BLE_DEV_NAME) != CHIP_NO_ERROR)
        appError(CHIP_ERROR_INTERNAL);

    gExampleDeviceInfoProvider.SetStorageDelegate(&chip::Server::GetInstance().GetPersistentStorage());
    chip::DeviceLayer::SetDeviceInfoProvider(&gExampleDeviceInfoProvider);

    chip::DeviceLayer::PlatformMgr().LockChipStack();
    // Initialize device attestation config
#ifdef EFR32_ATTESTATION_CREDENTIALS
    SetDeviceAttestationCredentialsProvider(SILABS::GetSILABSDacProvider());
#else
    SetDeviceAttestationCredentialsProvider(Examples::GetExampleDACProvider());
#endif

    chip::DeviceLayer::PlatformMgr().UnlockChipStack();

    SILABS_LOG("Starting App Task");
    if (AppTask::GetAppTask().StartAppTask() != CHIP_NO_ERROR)
        appError(CHIP_ERROR_INTERNAL);

    SILABS_LOG("Starting LIN task");
    gWindowCovering.Init();

    SILABS_LOG("Starting FreeRTOS scheduler");
    sl_system_kernel_start();

    // Should never get here.
    chip::Platform::MemoryShutdown();
    SILABS_LOG("vTaskStartScheduler() failed");
    appError(CHIP_ERROR_INTERNAL);
}

void sl_button_on_change(const sl_button_t * handle)
{
    AppTask::GetAppTask().ButtonEventHandler(handle, sl_button_get_state(handle));
}

void emberAfWindowCoveringClusterInitCallback(chip::EndpointId endpoint)
{
    SILABS_LOG("Setting window covering delegate for EP %d", (uint8_t) endpoint);
    if (endpoint == 1) {
        chip::app::Clusters::WindowCovering::SetDefaultDelegate(endpoint, &gWindowCovering);
        chip::app::Clusters::WindowCovering::ConfigStatusUpdateFeatures(endpoint);
    }
}
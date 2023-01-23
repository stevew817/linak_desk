#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <app/clusters/window-covering-server/window-covering-delegate.h>

using namespace ::chip::app::Clusters::WindowCovering;

class WindowCoveringImpl : public Delegate
{
public:
    /* Initialize HW */
    void Init(void);

    /**
     * @brief
     *   This method adjusts window covering position so the physical lift/slide and tilt is at the target
     *   open/up position set before calling this method. This will happen as fast as possible.
     *
     *   @param[in]  type            window covering type.
     *
     *   @return CHIP_NO_ERROR On success.
     *   @return Other Value indicating it failed to adjust window covering position.
     */
    CHIP_ERROR HandleMovement(WindowCoveringType type);

    /**
     * @brief
     *   This method stops any adjusting to the physical tilt and lift/slide that is currently occurring.
     *
     *   @return CHIP_NO_ERROR On success.
     *   @return Other Value indicating it failed to stop any adjusting to the physical tilt and lift/slide that is currently
     * occurring..
     */
    CHIP_ERROR HandleStopMotion();

private:
  
};

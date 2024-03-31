
#pragma once

#include <o3de_ros_project/o3de_ros_projectTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace o3de_ros_project
{
    class o3de_ros_projectRequests
    {
    public:
        AZ_RTTI(o3de_ros_projectRequests, o3de_ros_projectRequestsTypeId);
        virtual ~o3de_ros_projectRequests() = default;
        // Put your public methods here
    };

    class o3de_ros_projectBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using o3de_ros_projectRequestBus = AZ::EBus<o3de_ros_projectRequests, o3de_ros_projectBusTraits>;
    using o3de_ros_projectInterface = AZ::Interface<o3de_ros_projectRequests>;

} // namespace o3de_ros_project

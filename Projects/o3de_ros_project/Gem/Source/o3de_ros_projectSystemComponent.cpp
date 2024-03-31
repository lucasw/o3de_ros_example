
#include <AzCore/Serialization/SerializeContext.h>

#include "o3de_ros_projectSystemComponent.h"

#include <o3de_ros_project/o3de_ros_projectTypeIds.h>

namespace o3de_ros_project
{
    AZ_COMPONENT_IMPL(o3de_ros_projectSystemComponent, "o3de_ros_projectSystemComponent",
        o3de_ros_projectSystemComponentTypeId);

    void o3de_ros_projectSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<o3de_ros_projectSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void o3de_ros_projectSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("o3de_ros_projectService"));
    }

    void o3de_ros_projectSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("o3de_ros_projectService"));
    }

    void o3de_ros_projectSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void o3de_ros_projectSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    o3de_ros_projectSystemComponent::o3de_ros_projectSystemComponent()
    {
        if (o3de_ros_projectInterface::Get() == nullptr)
        {
            o3de_ros_projectInterface::Register(this);
        }
    }

    o3de_ros_projectSystemComponent::~o3de_ros_projectSystemComponent()
    {
        if (o3de_ros_projectInterface::Get() == this)
        {
            o3de_ros_projectInterface::Unregister(this);
        }
    }

    void o3de_ros_projectSystemComponent::Init()
    {
    }

    void o3de_ros_projectSystemComponent::Activate()
    {
        o3de_ros_projectRequestBus::Handler::BusConnect();
    }

    void o3de_ros_projectSystemComponent::Deactivate()
    {
        o3de_ros_projectRequestBus::Handler::BusDisconnect();
    }
}

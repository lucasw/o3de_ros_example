
#pragma once

#include <AzCore/Component/Component.h>

#include <o3de_ros_project/o3de_ros_projectBus.h>

namespace o3de_ros_project
{
    class o3de_ros_projectSystemComponent
        : public AZ::Component
        , protected o3de_ros_projectRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(o3de_ros_projectSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        o3de_ros_projectSystemComponent();
        ~o3de_ros_projectSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // o3de_ros_projectRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };
}

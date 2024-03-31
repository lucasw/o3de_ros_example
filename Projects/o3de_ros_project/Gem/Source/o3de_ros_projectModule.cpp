
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "o3de_ros_projectSystemComponent.h"

#include <o3de_ros_project/o3de_ros_projectTypeIds.h>

namespace o3de_ros_project
{
    class o3de_ros_projectModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(o3de_ros_projectModule, o3de_ros_projectModuleTypeId, AZ::Module);
        AZ_CLASS_ALLOCATOR(o3de_ros_projectModule, AZ::SystemAllocator);

        o3de_ros_projectModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                o3de_ros_projectSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<o3de_ros_projectSystemComponent>(),
            };
        }
    };
}// namespace o3de_ros_project

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), o3de_ros_project::o3de_ros_projectModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_o3de_ros_project, o3de_ros_project::o3de_ros_projectModule)
#endif

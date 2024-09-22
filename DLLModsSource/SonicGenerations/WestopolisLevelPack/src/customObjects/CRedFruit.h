#pragma once
#include "../include/Glitter.h"
#include "../include/Types.h"

namespace Mod
{
    class CRedFruit : public Sonic::CObjectBase, public Sonic::CSetObjectListener
    {
    public:
        BB_SET_OBJECT_MAKE("RedFruit");

    private:
        static inline const char* RedFruit_ModelName = "cmn_obj_redfruit";
        static inline const char* RedFruitBroken_ModelName = "cmn_obj_redfruit_brk";
        static inline const char* RedFruit_EffectName = "ef_red_fruit_dust";
        static inline const char* RedFruitBroken_EffectName = "ef_red_fruit_explode";
        bool IsBroken = false;

    public:
        boost::shared_ptr<hh::mr::CSingleElement> m_spRenderable_RedFruit;
        boost::shared_ptr<hh::mr::CSingleElement> m_spRenderable_RedFruit_brk;
        Hedgehog::Base::CRefPtr<Sonic::CLocalLight> m_Light;

        bool SetAddRenderables(Sonic::CGameDocument* in_pGameDocument,
                               const boost::shared_ptr<Hedgehog::Database::CDatabase>& in_spDatabase);
        bool SetAddColliders(const boost::shared_ptr<Hedgehog::Database::CDatabase>& in_spDatabase);
        bool ProcessMessage(Hedgehog::Universe::Message& message, bool flag);
        void KillCallback();
    };

    BB_SET_OBJECT_MAKE_HOOK(CRedFruit);
}

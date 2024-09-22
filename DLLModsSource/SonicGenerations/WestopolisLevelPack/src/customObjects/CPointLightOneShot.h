#pragma once
#include "../include/Types.h"

namespace Mod
{
    class CPointLightOneShot : public Sonic::CObjectBase, public Sonic::CSetObjectListener
    {
    public:
        BB_SET_OBJECT_MAKE("PointLightOneShot");

    private:
        bool m_Active = false;
        Hedgehog::Base::CRefPtr<Sonic::CLocalLight> m_Light;

        float m_InternalTime = 0.0f;
        float m_PlaybackTime = 0.0f;

    public:
        float FlashTime = 0.3f;
        float StartTime = 0.5f;
        int EaseType = 0;

        float LightRangeNear = 2.0f;
        float LightRangeFar = 10.0f;

        float Brightness = 5.0f;
        CVector4 Color = {1.0f, 0.8f, 0.6f, 1.0f};

        void InitializeEditParam(Sonic::CEditParam& in_rEditParam);
        bool SetAddRenderables(Sonic::CGameDocument* in_pGameDocument,
                               const boost::shared_ptr<Hedgehog::Database::CDatabase>& in_spDatabase);
        void SetUpdateParallel(const Hedgehog::Universe::SUpdateInfo& in_rUpdateInfo);
    };

    BB_SET_OBJECT_MAKE_HOOK(CPointLightOneShot)
}

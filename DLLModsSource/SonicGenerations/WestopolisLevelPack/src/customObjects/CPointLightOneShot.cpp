#include "CPointLightOneShot.h"

namespace Mod
{
    void CPointLightOneShot::InitializeEditParam(Sonic::CEditParam& in_rEditParam)
    {
        PARAM_FLOAT(LightRangeNear);
        PARAM_FLOAT(LightRangeFar);
        PARAM_FLOAT(Brightness);
        PARAM_FLOAT(StartTime);
        PARAM_FLOAT(FlashTime);
        PARAM_INT(EaseType);
        PARAM_VEC(Color);
    }

    bool CPointLightOneShot::SetAddRenderables(Sonic::CGameDocument* in_pGameDocument,
                                               const boost::shared_ptr<Hedgehog::Database::CDatabase>& in_spDatabase)
    {
        m_Light = in_pGameDocument->m_pMember->m_spLightManager->AddLocalLight(
            m_spMatrixNodeTransform->m_Transform.m_Position,
            CVector4(0, 0, 0, 1),
            CVector4(0, 0, LightRangeNear, LightRangeFar)
        );

        return true;
    }

    void CPointLightOneShot::SetUpdateParallel(const Hedgehog::Universe::SUpdateInfo& in_rUpdateInfo)
    {
        if (m_InternalTime > FlashTime + StartTime) return;

        m_InternalTime += in_rUpdateInfo.DeltaTime;
        if (m_InternalTime < StartTime) return;

        m_PlaybackTime += in_rUpdateInfo.DeltaTime;

        float t = fminf(m_PlaybackTime / FlashTime, 1.0f);
        if (EaseType == 1)
        {
            t = 1.0f - t;
            t *= t;
        }
        else if (EaseType == 2)
        {
            t = 1.0f - t;
            t *= t * t;
        }

        float brightness = Brightness * fmaxf((1.0f - cosf(M_PI * 2.0f * t)) * 0.5f, 0.0f);
        CVector color3 = Color * brightness;
        CVector4 color = {color3.x(), color3.y(), color3.z(), 1};

        m_Light->m_Dirty |= m_Light->m_spLight->m_Color != color;
        m_Light->m_spLight->m_Color = color;
    }
}

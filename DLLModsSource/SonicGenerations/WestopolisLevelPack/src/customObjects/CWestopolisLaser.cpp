#include "CWestopolisLaser.h"

namespace Mod
{
    void CWestopolisLaser::InitializeEditParam(Sonic::CEditParam& in_rEditParam)
    {
        CObjSoundPoint::InitializeEditParam(in_rEditParam);

        PARAM_BOOL(IsInstant);
    }

    void CWestopolisLaser::AddCallback(const Hedgehog::Base::THolder<Sonic::CWorld>& in_rWorldHolder,
                                       Sonic::CGameDocument* in_pGameDocument,
                                       const boost::shared_ptr<Hedgehog::Database::CDatabase>& in_spDatabase)
    {
        CObjSoundPoint::AddCallback(in_rWorldHolder, in_pGameDocument, in_spDatabase);

        m_pGlitterPlayer = Sonic::CGlitterPlayer::Make(in_pGameDocument);

        // Add lights depending on the type
        if (IsInstant)
        {
            Light2Laser.m_Light = in_pGameDocument->m_pMember->m_spLightManager->AddLocalLight(
                m_spMatrixNodeTransform->m_Transform.m_Position,
                CVector4(0, 0, 0, 1),
                CVector4(0, 0, Light2Laser.RangeNear, Light2Laser.RangeFar)
            );

            m_pGlitterPlayer->PlayOneshot(m_spMatrixNodeTransform, "ef_stg202_laser_explosion_new", 1.0f, 1);
        }
        else
        {
            Light1Delay.m_Light = in_pGameDocument->m_pMember->m_spLightManager->AddLocalLight(
                m_spMatrixNodeTransform->m_Transform.m_Position,
                CVector4(0, 0, 0, 1),
                CVector4(0, 0, Light1Delay.RangeNear, Light1Delay.RangeFar)
            );
            Light1Laser.m_Light = in_pGameDocument->m_pMember->m_spLightManager->AddLocalLight(
                m_spMatrixNodeTransform->m_Transform.m_Position,
                CVector4(0, 0, 0, 1),
                CVector4(0, 0, Light1Laser.RangeNear, Light1Laser.RangeFar)
            );
            
            m_pGlitterPlayer->PlayOneshot(m_spMatrixNodeTransform, "ef_stg202_laser_explosion_delay_new", 1.0f, 1);
        }

        // Add damage collision
        hk2010_2_0::hkpBoxShape* shapeDamage = new hk2010_2_0::hkpBoxShape(2.5, 50, 2.5);
        AddEventCollision((char*)0x0154DE02, shapeDamage, *(int*)0x01E0AFC4, true, m_spMatrixNodeTransform);
        shapeDamage->removeReference();
    }

    bool CWestopolisLaser::ProcessMessage(Hedgehog::Universe::Message& message, bool flag)
    {
        if (!flag)
        {
            return CObjSoundPoint::ProcessMessage(message, flag);
        }
        
        if (message.Is<Sonic::Message::MsgHitEventCollision>())
        {
            m_IsInsideDamageCollision = true;
            return true;
        }

        if (message.Is<Sonic::Message::MsgLeaveEventCollision>())
        {
            m_IsInsideDamageCollision = false;
            return true;
        }

        return CObjSoundPoint::ProcessMessage(message, flag);
    }

    void CWestopolisLaser::UpdateParallel(const Hedgehog::Universe::SUpdateInfo& in_rUpdateInfo)
    {
        CObjSoundPoint::UpdateParallel(in_rUpdateInfo);
        float deltaTime = in_rUpdateInfo.DeltaTime;
        bool shouldDealDamage;
        
        if (IsInstant)
        {
            Light2Laser.Flash(deltaTime);
            shouldDealDamage = false; // No damage in instant lasers
        }
        else
        {
            Light1Delay.Flash(deltaTime);
            Light1Laser.Flash(deltaTime);
            
            shouldDealDamage = Light1Laser.IsActive;
        }

        if (shouldDealDamage)
        {
            m_damageTime += deltaTime;
            if (m_damageTime <= m_maxDamageTime && m_IsInsideDamageCollision)
            {
                uint32_t sonicId = Sonic::Player::CPlayerSpeedContext::GetInstance()->m_pPlayer->m_ActorID;
                SendMessage(sonicId, boost::make_shared<Sonic::Message::MsgDamage>(*(int*)0x01E0BE28, m_spMatrixNodeTransform->m_Transform.m_Position));
            }
        }
    }

    void CWestopolisLaser::KillCallback()
    {
        m_pGlitterPlayer->m_pParticleController->m_ParticleMap.clear();
        Light1Delay.m_Light = nullptr;
        Light1Laser.m_Light = nullptr;
        Light2Laser.m_Light = nullptr;
    }

    void CWestopolisLaser::LightParam::Flash(float deltaTime)
    {
        if (m_InternalTime > FlashTime + StartTime)
        {
            IsActive = false;
            return;
        }

        m_InternalTime += deltaTime;
        if (m_InternalTime < StartTime)
        {
            IsActive = false;
            return;
        }
        IsActive = true;

        m_PlaybackTime += deltaTime;

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

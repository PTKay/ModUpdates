#include "CRedFruit.h"

namespace Mod
{
    bool CRedFruit::SetAddRenderables(Sonic::CGameDocument* in_pGameDocument,
                                      const boost::shared_ptr<Hedgehog::Database::CDatabase>& in_spDatabase)
    {
        hh::mr::CMirageDatabaseWrapper wrapper(in_spDatabase.get());

        m_spRenderable_RedFruit = boost::make_shared<hh::mr::CSingleElement>(wrapper.GetModelData(RedFruit_ModelName));
        m_spRenderable_RedFruit->BindMatrixNode(m_spMatrixNodeTransform);
        m_spRenderable_RedFruit_brk = boost::make_shared<hh::mr::CSingleElement>(
            wrapper.GetModelData(RedFruitBroken_ModelName));
        m_spRenderable_RedFruit_brk->BindMatrixNode(m_spMatrixNodeTransform);

        CGameObject::AddRenderable("Object", m_spRenderable_RedFruit, true);

        m_pGlitterPlayer = Sonic::CGlitterPlayer::Make(in_pGameDocument);
        m_pGlitterPlayer->PlayContinuous(m_pMember->m_pGameDocument, m_spMatrixNodeTransform, RedFruit_EffectName,
                                         0.25f);

        m_Light = in_pGameDocument->m_pMember->m_spLightManager->AddLocalLight(
            m_spMatrixNodeTransform->m_Transform.m_Position,
            CVector4(1, 0, 0, 1),
            CVector4(0, 0, 2, 4)
        );

        return true;
    }

    bool CRedFruit::SetAddColliders(const boost::shared_ptr<Hedgehog::Database::CDatabase>& in_spDatabase)
    {
        hk2010_2_0::hkpSphereShape* shapeDamage = new hk2010_2_0::hkpSphereShape(1);
        AddEventCollision((char*)0x0154DE02, shapeDamage, *(int*)0x01E0AFC4, true, m_spMatrixNodeTransform);
        shapeDamage->removeReference();

        return true;
    }

    void CRedFruit::KillCallback()
    {
        m_pGlitterPlayer->m_pParticleController->m_ParticleMap.clear();
        m_Light = nullptr;
    }

    bool CRedFruit::ProcessMessage(Hedgehog::Universe::Message& message, bool flag)
    {
        if (!flag)
        {
            return CObjectBase::ProcessMessage(message, flag);
        }

        if (!IsBroken && message.Is<Sonic::Message::MsgHitEventCollision>())
        {
            IsBroken = true;
            m_pGlitterPlayer->m_pParticleController->m_ParticleMap.clear();
            m_pGlitterPlayer->PlayOneshot(m_spMatrixNodeTransform, RedFruitBroken_EffectName, 0.25f, 1);
            Sonic::Player::CPlayerSpeedContext::GetInstance()->PlaySound(2002110, true);
            SendMessage(message.m_SenderActorID,
                        boost::make_shared<Sonic::Message::MsgDamage>(*(int*)0x01E0BE28,
                                                                      m_spMatrixNodeTransform->m_Transform.m_Position));

            CGameObject::RemoveRenderables();
            m_Light = nullptr;
            CGameObject::AddRenderable("Object", m_spRenderable_RedFruit_brk, true);

            return true;
        }

        return CObjectBase::ProcessMessage(message, flag);
    }
}


#include "include/Types.h"
#define _USE_LOG
#include "DebugDrawText.h"
#include <sstream>
#include <numbers>

namespace Mod
{

	struct SContactData
	{
		bool WasGrounded = false;
		Hedgehog::Math::CVector ContactNormal = Hedgehog::Math::CVector::UnitY();
		Hedgehog::Math::CVector v2 = Hedgehog::Math::CVector::Zero();
		Hedgehog::Math::CVector v3 = Hedgehog::Math::CVector::Zero();
		Hedgehog::Math::CVector v4 = Hedgehog::Math::CVector::Zero();
		Hedgehog::Math::CVector ContactPosition = Hedgehog::Math::CVector::Zero();
		int32_t someInt = 0;
		int32_t NumContacts = 0;
		int32_t someInt3 = 0;
		int32_t someInt4 = 0;
	};

	static struct
	{
		typedef Hedgehog::Math::CQuaternion CQuaternion;
		typedef Hedgehog::Math::CVector CVector;
		CQuaternion RotationA = CQuaternion::Identity();
		CQuaternion RotationB = CQuaternion::Identity();
		float InRotate = .0f;
		CVector VectorA;
		CVector VectorB;

	} sParams;

	HOOK(void, __cdecl, InitializeApplicationParams, 0x00D65180, Sonic::CParameterFile* This)
	{
		boost::shared_ptr<Sonic::CParameterGroup> parameterGroup = This->CreateParameterGroup("Tests", "Tests");

		/*
		Sonic::CEditParam* pc_TestRotation = parameterGroup->CreateParameterCategory("Rotation", "Test rotation for whatever the fuck this is");
		pc_TestRotation->CreateParamFloat(&sParams.RotationA.x(), "ROTA__X");
		pc_TestRotation->CreateParamFloat(&sParams.RotationA.y(), "ROTA__Y");
		pc_TestRotation->CreateParamFloat(&sParams.RotationA.z(), "ROTA__Z");
		pc_TestRotation->CreateParamFloat(&sParams.RotationA.w(), "ROTA__W");
		pc_TestRotation->CreateParamFloat(&sParams.RotationB.x(), "ROTB__X");
		pc_TestRotation->CreateParamFloat(&sParams.RotationB.y(), "ROTB__Y");
		pc_TestRotation->CreateParamFloat(&sParams.RotationB.z(), "ROTB__Z");
		pc_TestRotation->CreateParamFloat(&sParams.RotationB.w(), "ROTB__W");
		pc_TestRotation->CreateParamFloat(&sParams.InRotate, "RotateFloat");
		parameterGroup->Flush();
*/
		Sonic::CEditParam* pc_TestRotation = parameterGroup->CreateParameterCategory("VectorTests", "");
		pc_TestRotation->CreateParamFloat(&sParams.VectorA.x(), "VECA_X");
		pc_TestRotation->CreateParamFloat(&sParams.VectorA.y(), "VECA_Y");
		pc_TestRotation->CreateParamFloat(&sParams.VectorA.z(), "VECA_Z");
		pc_TestRotation->CreateParamFloat(&sParams.VectorB.x(), "VECB_X");
		pc_TestRotation->CreateParamFloat(&sParams.VectorB.y(), "VECB_Y");
		pc_TestRotation->CreateParamFloat(&sParams.VectorB.z(), "VECB_Z");
		parameterGroup->Flush();

		originalInitializeApplicationParams(This);
	}

	bool CheckLandAngleValid(Sonic::Player::CPlayerSpeedContext* in_pContext, const Hedgehog::Math::CVector& in_rVectorA, const Hedgehog::Math::CVector& in_rVectorB)
	{
		if (in_pContext->StateFlag(eStateFlag_WallWalkJump))
			return true;

		const float landAngle = in_pContext->m_spParameter->Get<float>(Sonic::Player::ePlayerSpeedParameter_LandEnableMaxSlope)
			+ !in_pContext->m_Grounded ? 0.0f : 2.5f;

		if (Hedgehog::Math::CVector::Dot(in_rVectorA, in_rVectorB) < cos(landAngle * DEG2RADf))
			return false;

		return true;
	}

	Hedgehog::Math::CVector ConvertToSpherePolar(const Hedgehog::Math::CVector& in_rVector)
	{
		const float HalfPiZ = M_PI * 0.5 + in_rVector.z();
		const float sin_HalfPiZ = sin(HalfPiZ);

		const float sin_Y = sin(in_rVector.y());
		const float cos_Y = cos(in_rVector.y());

		const float cos_HalfPiZ = cos(HalfPiZ);

		const Hedgehog::Math::CVector result (sin_Y * in_rVector.x() * sin_HalfPiZ,
		                                              in_rVector.x() * cos_HalfPiZ,
		                                      cos_Y * in_rVector.x() * sin_HalfPiZ);

		return result;
	}

	void __cdecl AngleWrap(float* a1, float a2)
	{
		using namespace std::numbers;

		if (*a1 - a2 <= pi)
		{
			if (a2 - *a1 > pi)
				*a1 = *a1 + (pi * 2.0);
		}
		else
		{
			*a1 = *a1 - (pi * 2.0);
		}
	}

	static constexpr int pCollectPoints = 0x010E10F0;
	bool CollectPoints(Sonic::CCharacterProxy* in_pProxy, const Hedgehog::Math::CQuaternion& in_rRotation,
		bool in_Condition, float in_SearchDistance, unsigned int in_Flag, Hedgehog::Math::CVector* out_pNormal = nullptr)
	{
		bool result = false;
		__asm
		{
			push out_pNormal
			push in_Flag
			push in_SearchDistance
			push in_Condition
			push in_pProxy
			mov eax, in_rRotation
			call[pCollectPoints]
		}
	}

	HOOK(void, __stdcall, _MainCollisionFunc, 0x00E32180, Sonic::Player::CPlayerSpeedPostureCommon* in_pPosture, const Hedgehog::Math::CVector& in_rVelocity)
	{
		using namespace hh::math;
		using namespace Sonic::Player;

		const float deltaTime = in_pPosture->GetDeltaTime();
		const auto context = static_cast<CPlayerSpeedContext*>(in_pPosture->GetContext());
		boost::shared_ptr<Sonic::CMatrixNodeTransform>& node = context->m_spMatrixNode;


		//sParams.VectorA = sParams.VectorA.normalizedSafe();
		//sParams.VectorB = sParams.VectorB.normalizedSafe();

		std::stringstream stream;
		//stream << sParams.VectorA;
		//DebugDrawText::log(stream.str().c_str());
		//stream.str("");
		//stream << sParams.VectorB;
		//DebugDrawText::log(stream.str().c_str());

		FUNCTION_PTR(void, __cdecl, sub_6F01F0, 0x6F01F0, CVector * out_pVec, const CVector& in_rVec);
		CVector vec = CVector::Identity();
		sub_6F01F0(&vec, sParams.VectorA);

		stream << vec;
		DebugDrawText::log(stream.str().c_str());

		original_MainCollisionFunc(in_pPosture, in_rVelocity);
		return;

		auto ApplyVelocity = [&node, context, deltaTime](const CVector& velocity)
		{
			node->m_Transform.SetPosition(node->m_Transform.m_Position + (velocity * deltaTime));
			node->NotifyChanged();

			context->m_Velocity = velocity;
			context->m_VelocityChanged = true;
			context->m_HorizontalOrVerticalVelocityChanged = false;
		};

		Sonic::CCharacterProxy* proxy = context->m_spCharacterProxy.get();

		if (context->StateFlag(eStateFlag_IgnoreTerrain))
		{
			ApplyVelocity(in_rVelocity);
			return;
		}

		if (in_rVelocity.squaredNorm() < 0.0000010000001f)
			return;

		const CQuaternion pitchroll = context->GetPitchRollRotation(context->m_Grounded);
		struct
		{
			bool grounded;
			bool wasGrounded;
			bool onNormalGround;
			CVector position;
			CVector upDirection;
		}
		const initialConditions(
			context->m_Grounded,
			context->m_WasGrounded,
			context->m_pStateFlag->m_Flags[CPlayerSpeedContext::eStateFlag_OnNoWallWalkGround],
			node->m_Transform.m_Position,
			context->m_UpVector
		);

		context->StateFlag(eStateFlag_OnNoWallWalkGround) = false;

		SContactData contactData;

		const CVector upDirection = context->StateFlag(eStateFlag_InvokePtmSpike)
			? context->m_UpVector : CVector::Up();

		CVector localVelocity = in_rVelocity;
		in_pPosture->ChangeVelocity(&localVelocity);

		proxy->m_DamageID = 0;
		proxy->m_NormalStrength = context->m_NormalStrength;
		proxy->m_VelocityStrengthScale = context->m_spParameter->Get<float>(ePlayerSpeedParameter_VelocityStrengthScale);

		const float angle = [=]() -> float
		{
			if (!initialConditions.grounded)
			{
				if (context->StateFlag(eStateFlag_WallWalkJump))
					return 90.0f * DEG2RADf;
			}
			else if (CheckLandAngleValid(context, upDirection, context->m_UpVector))
				return 90.0f * DEG2RADf;

			return 60.0f * DEG2RADf;
		}();

		proxy->cosAngleThing = cos(angle);
		proxy->m_Velocity = localVelocity;
		proxy->m_Position = initialConditions.position;
		proxy->m_UpAxis = initialConditions.upDirection;

		CollectPoints(proxy, pitchroll, true, 1.0f, 1);
		if (!context->m_Grounded)
		{
			context->m_spMatrixNode->m_Transform.SetRotation(proxy->m_Rotation);
			context->m_spMatrixNode->NotifyChanged();
			context->CleanYawRotation();
		}

		CVector workingPosition = initialConditions.position;
		if (initialConditions.grounded)
		{
			const CVector contactNormal = context->m_FloorAndGrindRailNormal;
			const float rigidBodyOffsetDistance = context->m_RigidBodyOffsetDistance;
			CVector rhs = contactNormal * rigidBodyOffsetDistance;

			proxy->m_Position = initialConditions.position;
			proxy->var130 = rhs * (1.0 / deltaTime);
			proxy->Integrate(deltaTime);
			workingPosition = proxy->GetPosition();
			context->m_RigidBodyOffsetDistance = CVector::Dot(workingPosition - initialConditions.position, context->m_FloorAndGrindRailNormal);
		}

		CVector tempVelocity = in_rVelocity;
		if (initialConditions.wasGrounded)
		{
			const float fallSpeed = fminf(CVector::Dot(in_rVelocity, context->m_FloorAndGrindRailNormal), 0.0f);
			const CVector downwardForce = context->m_FloorAndGrindRailNormal * fallSpeed;
			tempVelocity -= downwardForce;
		}
		if (tempVelocity.squaredNorm() < 0.0025000002)
			tempVelocity = CVector::Zero();

		proxy->m_UpAxis = initialConditions.upDirection;
		proxy->m_Position = workingPosition;
		proxy->var130 = tempVelocity; // Is this velocity minus gravity...? Huh.
		proxy->Integrate(deltaTime);

		CVector bodyForce = proxy->var130;
		if (bodyForce.squaredNorm() > tempVelocity.squaredNorm())
			bodyForce = bodyForce.normalizedSafe() * tempVelocity.norm();


		bool applyBouyantForce = false;

		if (context->m_VelocityChanged)
			context->HandleVelocityChanged();
		proxy->ScanContacts(&contactData, deltaTime, context->m_HorizontalVelocity);


		// Temp fallback
		ApplyVelocity(in_rVelocity);
	}

	static Sonic::CPlayer2DNormalCamera::SGlobalParams& s_GlobalParam = *reinterpret_cast<Sonic::CPlayer2DNormalCamera::SGlobalParams*>(0x01E5E708);

	void Camera2D_AttemptCleanBroken(Sonic::CPlayer2DNormalCamera* This)
	{
		using namespace hh::math;
		DebugDrawText::log("Camera2D");

		Sonic::CCamera* pCamera = This->GetContext();
		const float deltaTime = This->GetDeltaTime();
		const float frameDeltaTime = deltaTime / 60.0f;


		struct
		{
			float distance = 0;
			CVector position;
			CVector up;
			CVector forward;
		} pathPoint;

		struct : Sonic::Message::MsgGetGroundInfo
		{
			CVector velocity;
			CVector targetPosition;
			CVector frontDirection;
			float targetPositionOffset = 0;
			bool unknown = false;
		} playerInfo = {};
		
		pCamera->SendMessageImm(pCamera->m_ActorID, static_cast<Sonic::Message::MsgGetGroundInfo>(playerInfo));
		pCamera->SendMessageImm(pCamera->m_ActorID, Sonic::Message::MsgGetVelocity(&playerInfo.velocity));
		pCamera->SendMessageImm(pCamera->m_ActorID, Sonic::Message::MsgGetFrontDirection(&playerInfo.frontDirection));
		{
			Sonic::Message::MsgGetCameraTargetPosition msgTargetPosition(&playerInfo.targetPosition, true);
			pCamera->SendMessageImm(pCamera->m_ActorID, msgTargetPosition);
			playerInfo.targetPositionOffset = msgTargetPosition.m_Offset;
			playerInfo.unknown = msgTargetPosition.m_boolA;
		}
		const CVector cameraFloorVector = playerInfo.m_Distance <= (double)s_GlobalParam.TargetUpMaxGroundDistance
		                                ? playerInfo.m_GroundNormal
		                                : CVector::Up();

		if (!pCamera->SendMessageImm(pCamera->m_ActorID, Sonic::Message::MsgGet2DPathPNT(&pathPoint.distance, &pathPoint.position, &pathPoint.up, &pathPoint.forward)))
		{
			// Interesting. Fallback seems to be just keeping the camera locked in place if there's no path?
			// What'd probably be a neat change: If we don't have a path, just assume the camera's current forward position is perpendicular to a "path."
			// Then work around that.
			// TODO: Implement this, sounds interesting.

			This->m_CameraPosition1 = pCamera->m_Position;
			This->m_CameraPosition2 = This->m_CameraPosition1;

			This->m_UpVector1 = CVector::Up();
			This->m_UpVector2 = This->m_UpVector1;

			This->m_TargetPosition1 = pCamera->m_TargetPosition;
			This->m_TargetPosition2 = This->m_TargetPosition1;

			return;
		}

		This->m_pParams->BaseSpacePathPosition = pathPoint.distance;
		This->m_PointPosition = pathPoint.position;

		Sonic::Message::MsgGetForCamera2DState msgCamState{};
		pCamera->SendMessageImm(pCamera->m_ActorID, msgCamState);

		if (CVector::Dot(pathPoint.up,      This->m_2DPathUp) <= -0.25
		|| (CVector::Dot(pathPoint.forward, This->m_2DPathFwd) >= -0.25))
		{
			This->m_2DPathFwd = pathPoint.forward;
		}
		else
		{
			This->m_2DPathFwd = -pathPoint.forward;
		}
		This->m_2DPathUp = pathPoint.up;

		CVector cross = msgCamState.m_IsValid ? CVector::Cross(This->m_2DPathFwd,  This->m_2DPathUp)
		                                      : CVector::Cross(This->m_2DPathFwd, -This->m_2DPathUp);

		This->m_Velocity += (playerInfo.velocity - This->m_Velocity) * std::fminf(s_GlobalParam.PlayerVelocitySensitive * frameDeltaTime, 1.0f);
		if (playerInfo.m_OnGround)
		{
			This->m_Velocity -= playerInfo.m_GroundNormal * CVector::Dot(This->m_Velocity,    playerInfo.m_GroundNormal);
			This->m_Velocity += playerInfo.m_GroundNormal * CVector::Dot(playerInfo.velocity, playerInfo.m_GroundNormal);;
		}
		This->m_SphericalPosition += This->m_Velocity * deltaTime;
		This->m_SphericalPosition += (playerInfo.targetPosition - This->m_SphericalPosition) * s_GlobalParam.SphericalPositionSensitiveZ * frameDeltaTime;

		CVector TargetPosition = This->m_pParams->IsPositionBasePlayer ? This->m_SphericalPosition : This->m_PointPosition;
		const CVector CameraVerticalVelocity   = cameraFloorVector * CVector::Dot(This->m_Velocity, cameraFloorVector);
		      CVector CameraHorizontalVelocity = This->m_Velocity - CameraVerticalVelocity;

		CVector velocityDirection = CameraHorizontalVelocity.squaredNorm() > 1.0
		                          ? CameraHorizontalVelocity.normalized()
		                          : playerInfo.frontDirection;

		CVector CameraDirection = CVector::Dot(velocityDirection, This->m_2DPathFwd) <= 0.0
		            ? -This->m_2DPathFwd
		            :  This->m_2DPathFwd;

		if (This->m_pParams->IsPositionBasePlayer)
		{
			const auto* pParams = This->m_pParams;
			const float frontVelocity = std::fmaxf(CVector::Dot(This->m_Velocity, CameraDirection), 0.0f);
			float v33 = This->m_pParams->TargetFrontOffsetSpeedScale * frontVelocity + This->m_pParams->TargetFrontOffset;
			if ((double)v33 > pParams->TargetFrontOffsetMax)
				v33 = pParams->TargetFrontOffsetMax;
			float scalara = playerInfo.targetPositionOffset + v33;
			CVector v92 = CameraDirection * scalara;
			CVector v34 = playerInfo.unknown ? -v92 : v92;

			This->m_TargetFrontOffset += (v34 - This->m_TargetFrontOffset) * fminf(s_GlobalParam.TargetFrontOffsetSensitive * frameDeltaTime, 1.0f);
			TargetPosition += This->m_TargetFrontOffset;

			// Gens does an if check again with the same bool. Mistake?

			if (playerInfo.m_OnGround)
				This->m_TargetUpGroundDistance += This->m_Velocity.y() * deltaTime;

			float groundDistance = playerInfo.m_Distance;
			if (groundDistance > (double)s_GlobalParam.TargetUpMaxGroundDistance)
				groundDistance = s_GlobalParam.TargetUpMaxGroundDistance;
			float slopeSensitiveAir = s_GlobalParam.SlopeSensitiveAir * frameDeltaTime;
			if (slopeSensitiveAir > 1.0)
				slopeSensitiveAir = 1.0;

			const float targetUpMove = TargetPosition.y() - groundDistance * s_GlobalParam.TargetUpMoveRate;
			This->m_TargetUpGroundDistance += (targetUpMove - This->m_TargetUpGroundDistance) * slopeSensitiveAir;

			//FUNCTION_PTR(void, __thiscall, UnknownVectorMath, 0x006F2180, const CMatrix& in_rMatrix, CVector* out_pVector, const CVector& in_rBaseVector);
			auto MakeMatrixVector = [](const CMatrix& in_rMatrix, const CVector& in_rBaseVector) -> CVector
			{
				FUNCTION_PTR(void, __thiscall, UnknownVectorMath, 0x006F2180, const CMatrix & _in_rMatrix, CVector * out_pVector, const CVector & _in_rBaseVector);
				CVector out = {};
				UnknownVectorMath(in_rMatrix, &out, in_rBaseVector);
				return out;
			};

			const CMatrix projectedViewMatrix = *(CMatrix*)&pCamera->m_MyCamera.m_Projection * pCamera->m_MyCamera.m_View;
			const CVector halfScreen = CVector(0.5, 0.5, 0);

			const CVector SphereProjectionVectorA = MakeMatrixVector(projectedViewMatrix, This->m_SphericalPosition)
			                                      * 0.5f + halfScreen;
			const CVector SphereProjectionVectorB = MakeMatrixVector(projectedViewMatrix, This->m_SphericalPosition + pCamera->m_MyCamera.m_View.GetVectorFromRow(1))
			                                      * 0.5f + halfScreen;

			const float VectorDifference = SphereProjectionVectorB.y() - SphereProjectionVectorA.y();
			const float offsetDifference = pParams->TargetUpOffset * VectorDifference;
			const float diffSlopeSensitive = (s_GlobalParam.SlopeSensitiveVelocityScaleOffset + offsetDifference) / VectorDifference;
			const float diffTargetUpOffset = (s_GlobalParam.TargetUpMaxOffsetPositive - offsetDifference) / VectorDifference;

			// Lambda to avoid some gotos.
			[&]
			{
				if (This->m_TargetUpGroundDistance - This->m_SphericalPosition.y() <= diffTargetUpOffset)
				{
					if (This->m_SphericalPosition.y() - This->m_TargetUpGroundDistance <= diffSlopeSensitive)
					{
						TargetPosition.y() = This->m_TargetUpGroundDistance;
						return;
					}
					This->m_TargetUpGroundDistance = This->m_SphericalPosition.y() - diffSlopeSensitive;
				}
				else
				{
					This->m_TargetUpGroundDistance = This->m_SphericalPosition.y() + diffTargetUpOffset;
				}
				TargetPosition.y() = This->m_TargetUpGroundDistance;
			}();
		}

		float sensitive = s_GlobalParam.SlopeSensitiveVelocityScale;
		if (playerInfo.m_OnGround)
		{
			float pCameraa = This->m_Velocity.norm() - s_GlobalParam.TargetUpOffsetSensitive;
			if (pCameraa < 0.0)
				pCameraa = 0.0;

			const float v50 = pCameraa * s_GlobalParam.SlopeSensitiveMax + s_GlobalParam.TargetUpMaxOffsetNegative;
			if (v50 <= (double)s_GlobalParam.SphericalPositionSensitiveY)
				sensitive = v50;
			else
				sensitive = s_GlobalParam.SphericalPositionSensitiveY;
		}

		const CQuaternion interpolatedRotation =
			CQuaternion::Slerp(CQuaternion::FromAxes(CVector(0, 1, 0), This->m_Field180,  CVector(1, 0, 0)),
			                   CQuaternion::FromAxes(CVector(0, 1, 0), cameraFloorVector, CVector(1, 0, 0)),
			                   fminf(sensitive * deltaTime, 1.0f));

		This->m_Field180 = interpolatedRotation.ToRotationMatrix().TransformVector(CVector(0, 1, 0));
		TargetPosition += This->m_Field180 * This->m_pParams->TargetUpOffset;

		CVector sphericalPositionTemp = This->m_SphericalPosition;

		const float crossLengthSquared = CVector2(cross.x(), cross.z()).squaredNorm();
		if (crossLengthSquared >= 0.0099999998)
			sphericalPositionTemp.z() = (double)sphericalPositionTemp.z() - atan2(cross.y(), crossLengthSquared) * s_GlobalParam.SlopeRollRate;
		else
			sphericalPositionTemp.z() = sphericalPositionTemp.z() - s_GlobalParam.SlopeRollRate * 1.570796370506287;

		sphericalPositionTemp.y() = atan2(cross.x(), cross.z()); + sphericalPositionTemp.y();
		AngleWrap(&sphericalPositionTemp.y(), This->m_AngularPosition.y());
		AngleWrap(&sphericalPositionTemp.z(), This->m_AngularPosition.z());
		This->m_AngularPosition.x() = sphericalPositionTemp.x();
		float v59 = s_GlobalParam.PlayerPositionSensitive * frameDeltaTime;
		if (v59 > 1.0)
			v59 = 1.0;
		float v60 = (sphericalPositionTemp.y() - This->m_AngularPosition.y()) * v59 + This->m_AngularPosition.y();
		This->m_AngularPosition.y() = v60;
		float v61 = s_GlobalParam.SlopeSensitive * frameDeltaTime;
		if (v61 > 1.0)
			v61 = 1.0;
		float v62 = (sphericalPositionTemp.z() - This->m_AngularPosition.z()) * v61 + This->m_AngularPosition.z();
		This->m_AngularPosition.z() = v62;
		float v63 = v60;
		if (v63 > M_PI)
		{
			This->m_AngularPosition.y() = v63 - M_PI * 2.0;
		}
		else if ((float)-M_PI > v63)
		{
			This->m_AngularPosition.y() = M_PI * 2.0 + This->m_AngularPosition.y();
		}

		CVector v115 = ConvertToSpherePolar(This->m_AngularPosition);
		float pCamerab = fabs(CVector::Dot(v115.normalizedSafe(), CVector::Up()));
		CVector v67;
		if (pCamerab <= 0.949999988079071)
		{
			v67 = CVector::Up();
		}
		else
		{
			float v66 = (pCamerab - 0.949999988079071) / 0.05000000074505806;
			CVector a3 = CVector::Up() * (1.0 - v66);
			CVector a2 = This->m_Field180 * v66;
			v67 = a2 + a3;
		}


		TargetPosition = Sonic::Player::CPlayerSpeedContext::GetInstance()->m_spMatrixNode->m_Transform.m_Position;

		This->m_CameraPosition1 = TargetPosition + CVector(0,0,30);
		This->m_UpVector1 = v67;
		This->m_TargetPosition1 = TargetPosition;

		This->m_CameraPosition2 = This->m_CameraPosition1;
		This->m_UpVector2 = cameraFloorVector;
		This->m_TargetPosition2 = This->m_TargetPosition1;

		//This->m_CameraPosition2 = This->m_SphericalPosition + cross;
		//This->m_UpVector2 = cameraFloorVector;
		//This->m_TargetPosition2 = This->m_SphericalPosition;
	}

	void Camera2D_GroundTruth(Sonic::CPlayer2DNormalCamera* This)
	{
		using namespace Hedgehog::Math;
		using namespace Sonic::Message;
		using namespace Sonic;

		// HACKS
#define vector_subtract(a,b) (a - b)
#define vector_add(a,b) (a + b)
#define _mm_mul_ps(a,b) (a * b)

		// Funcs
		auto MakeMatrixVector = [](const CMatrix& in_rMatrix, const CVector& in_rBaseVector) -> CVector
		{
			FUNCTION_PTR(void, __thiscall, UnknownVectorMath, 0x006F2180, const CMatrix & _in_rMatrix, CVector * out_pVector, const CVector & _in_rBaseVector);
			CVector out = {};
			UnknownVectorMath(in_rMatrix, &out, in_rBaseVector);
			return out;
		};
		auto MakeMatrix44Vector = [](const CMatrix44& in_rMatrix, const CVector& in_rBaseVector) -> CVector
		{
			FUNCTION_PTR(void, __thiscall, UnknownVectorMath, 0x006F2180, const CMatrix44 & _in_rMatrix, CVector * out_pVector, const CVector & _in_rBaseVector);
			CVector out = {};
			UnknownVectorMath(in_rMatrix, &out, in_rBaseVector);
			return out;
		};

		FUNCTION_PTR(void, __cdecl, MatMultiply, 0x009C28C0, CMatrix44 * pOut, CMatrix44 * pA1, CMatrix * pA2);

		bool v6; // zf
		CVector v8; // xmm2
		CVector v9; // xmm2
		CVector* v10; // esi
		CVector* v11; // edi
		CVector v12; // xmm0
		CVector* v13; // edx
		CVector* v14; // eax
		CVector* v15; // ecx
		float v16; // xmm0_4
		CVector* v17; // esi
		const CVector* v18; // eax
		CVector* v19; // edi
		CVector* v20; // edi
		double v21; // xmm1_8
		float v22; // xmm0_4
		CVector v23; // xmm0
		CPlayer2DNormalCamera::SParams* v24; // eax
		CVector* v26; // esi
		CVector v27; // xmm0
		CPlayer2DNormalCamera::SParams* v28; // edx
		float v29; // xmm0_4
		CPlayer2DNormalCamera::SParams* v30; // eax
		float v31; // xmm0_4
		CVector v32; // xmm0
		float v33; // xmm1_4
		float* v34; // eax
		float v35; // xmm0_4
		float v36; // xmm2_4
		float v37; // xmm1_4
		CVector* v38; // eax
		CPlayer2DNormalCamera::SParams* v39; // edx
		float v40; // xmm1_4
		double v41; // xmm0_8
		float v42; // xmm3_4
		double v43; // xmm2_8
		float v44; // xmm0_4
		float v45; // xmm1_4
		double v46; // xmm0_8
		float v47; // xmm1_4
		float v48; // xmm1_4
		float v49; // xmm0_4
		CMatrix44* v50; // eax
		CPlayer2DNormalCamera::SParams* v51; // eax
		float v52; // xmm1_4
		float v53; // xmm0_4
		double v54; // st7
		double v55; // st7
		float v56; // xmm2_4
		float v57; // xmm1_4
		float v58; // xmm0_4
		float v59; // xmm1_4
		float v60; // xmm1_4
		double v61; // xmm0_8
		double v62; // xmm0_8
		CVector* v63; // eax
		float v64; // xmm0_4
		CVector v65; // xmm1
		CVector v66; // xmm2
		CVector v67; // xmm0
		const CVector* v68; // [esp+0h] [ebp-218h]
		float v69; // [esp+0h] [ebp-218h]
		float v70; // [esp+0h] [ebp-218h]
		float v71; // [esp+0h] [ebp-218h]
		float v72; // [esp+0h] [ebp-218h]
		int actorID; // [esp+18h] [ebp-200h]
		float v74; // [esp+18h] [ebp-200h]
		CCamera* pCamera; // [esp+1Ch] [ebp-1FCh]
		float actorIDa; // [esp+1Ch] [ebp-1FCh]
		float actorIDb; // [esp+1Ch] [ebp-1FCh]
		float actorIDc; // [esp+1Ch] [ebp-1FCh]
		bool v79; // [esp+23h] [ebp-1F5h]
		CVector v80; // [esp+24h] [ebp-1F4h] BYREF
		float v81; // [esp+3Ch] [ebp-1DCh] BYREF
		bool v82; // [esp+40h] [ebp-1D8h]
		CVector v83; // [esp+44h] [ebp-1D4h] BYREF
		CVector lineNumber_12; // [esp+54h] [ebp-1C4h] BYREF
		CVector msg_12; // [esp+64h] [ebp-1B4h] BYREF
		float v86; // [esp+80h] [ebp-198h]
		CVector* a2; // [esp+88h] [ebp-190h]
		boost::shared_ptr<MsgGetGroundInfo> v89; // [esp+8Ch] [ebp-18Ch] BYREF
		CVector a1_4; // [esp+94h] [ebp-184h] BYREF
		boost::shared_ptr<MsgGetCameraTargetPosition> v91; // [esp+ACh] [ebp-16Ch] BYREF
		CVector result_4; // [esp+B4h] [ebp-164h] BYREF
		CVector result_12; // [esp+C4h] [ebp-154h] BYREF
		CVector v94; // [esp+D4h] [ebp-144h] BYREF
		CVector out_pResult_12; // [esp+E4h] [ebp-134h] BYREF
		CVector pBaseSpacePathPosition; // [esp+104h] [ebp-114h] BYREF
		CVector v100; // [esp+114h] [ebp-104h] BYREF
		CVector v101; // [esp+124h] [ebp-F4h] BYREF
		CVector v102; // [esp+134h] [ebp-E4h] BYREF
		CVector v103; // [esp+144h] [ebp-D4h] BYREF
		CMatrix44 This_12; // [esp+154h] [ebp-C4h] BYREF

		pCamera = This->GetContext();
		actorID = pCamera->m_ActorID;
		*(float*)&a2 = This->GetDeltaTime();
		v86 = *(float*)&a2 * 60.0;


		v89 = boost::make_shared<MsgGetGroundInfo>();
		v91 = boost::make_shared<MsgGetCameraTargetPosition>(&v100);

		boost::shared_ptr<MsgGetForCamera2DState> msgForCam2DState = boost::make_shared<MsgGetForCamera2DState>();
		boost::shared_ptr<MsgGet2DPathPNT> spMsgPathPNT =
			boost::make_shared<MsgGet2DPathPNT>(&v81, &This->m_PointPosition, &msg_12, &a1_4);


		v91->m_Offset = 0.0f;
		v91->m_boolA = false;
		v91->m_NoOffset = true;
		v91->m_UseModelMatrix = true;

		pCamera->SendMessageImm(actorID, v91);
		pCamera->SendMessageImm(actorID, boost::make_shared<MsgGetVelocity>(&result_12));
		pCamera->SendMessageImm(actorID, boost::make_shared<MsgGetFrontDirection>(&v101));
		pCamera->SendMessageImm(actorID, v89);
		
		pBaseSpacePathPosition.x() = 0.0;
		pBaseSpacePathPosition.y() = 1.0;
		pBaseSpacePathPosition.z() = 0.0;

		if (v89->m_Distance <= (double)s_GlobalParam.TargetUpMaxGroundDistance)
		{
			out_pResult_12 = v89->m_GroundNormal;
		}
		else
		{
			out_pResult_12.x() = 0.0;
			out_pResult_12.y() = 1.0;
			out_pResult_12.z() = 0.0;
		}
		v6 = !This->m_pParams->IsBaseSpacePlayer;
		v82 = false;
		if (v6)
			goto LABEL_20;

		if (!pCamera->SendMessageImm(actorID, spMsgPathPNT))
		{
			v8 = pCamera->m_TargetPosition;
			This->m_CameraPosition1 = pCamera->m_Position;
			v80.y() = 1.0;
			v80.x() = 0.0;
			v80.z() = 0.0;
			This->m_UpVector1 = v80;
			This->m_TargetPosition1 = v8;
			v9 = pCamera->m_TargetPosition;
			v80.y() = 1.0;
			This->m_CameraPosition2 = pCamera->m_Position;
			v80.x() = 0.0;
			v80.z() = 0.0;
			This->m_UpVector2 = v80;
			This->m_TargetPosition2 = v9;
			return;
		}

		v79 = pCamera->SendMessageImm(actorID, msgForCam2DState);
		if (v79)
			v82 = msgForCam2DState->m_IsValid;
		This->m_pParams->BaseSpacePathPosition = v81;
		v10 = &This->m_2DPathUp;
		if (CVector::Dot((CVector*)&msg_12, &This->m_2DPathUp) <= -0.25
			|| (v11 = &This->m_2DPathFwd,
				CVector::Dot((CVector*)&a1_4, &This->m_2DPathFwd) >= -0.25))
		{
			v12 = a1_4;
			v11 = &This->m_2DPathFwd;
		}
		else
		{
			v80 = -a1_4;
			v12 = v80;
		}
		*v11 = v12;
		*v10 = msg_12;
		if (!(v82))
		{
		LABEL_20:
			v15 = &This->m_2DPathFwd;
			v13 = &This->m_2DPathUp;
			v14 = &result_4;
		}
		else
		{
			a1_4 = -This->m_2DPathUp;
			v13 = &a1_4;
			v14 = &v94;
			v15 = v11;
		}
		a1_4 = *(CVector*)CVector::Cross(v15, v14, v13);
		v16 = s_GlobalParam.PlayerVelocitySensitive * v86;
		if (v16 > 1.0)
			v16 = 1.0;
		v17 = (CVector*) & This->m_Velocity;
		msg_12 = vector_subtract((CVector)result_12, (CVector)This->m_Velocity);
		CVector::Multiply(&v80, (CVector*)&msg_12, v16);
		v18 = (CVector*)v89.get();
		This->m_Velocity = vector_add((CVector)This->m_Velocity, (CVector)v80);
		if (LOBYTE(v18[2].y()))
		{
			v19 = (CVector*)&v18[1];
			v81 = CVector::Dot(&This->m_Velocity, v18 + 1);
			CVector::Multiply((CVector*)&msg_12, v19, v81);
			v20 = (CVector*)((char*)v89.get() + 16);
			v68 = (const CVector*)((char*)v89.get() + 16);
			*v17 = vector_subtract(*v17, msg_12);
			v81 = CVector::Dot(&result_12, v68);
			CVector::Multiply((CVector*)&msg_12, v20, v81);
			*v17 = vector_add(*v17, msg_12);
		}
		result_12 = *v17;
		CVector::Multiply((CVector*)&msg_12, &result_12, *(float*)&a2);
		v21 = v86;
		This->m_SphericalPosition = vector_add((CVector)This->m_SphericalPosition, msg_12);
		v22 = s_GlobalParam.SphericalPositionSensitiveZ * v21;
		if (v22 > 1.0)
			v22 = 1.0;
		msg_12 = vector_subtract(v100, (CVector)This->m_SphericalPosition);
		CVector::Multiply(&v80, (CVector*)&msg_12, v22);
		This->m_SphericalPosition = vector_add((CVector)This->m_SphericalPosition, (CVector)v80);
		v23 = This->m_SphericalPosition;
		v24 = This->m_pParams;
		v100 = (CVector)v23;
		if (!v24->IsPositionBasePlayer)
			v23 = This->m_PointPosition;
		lineNumber_12 = v23;
		v81 = CVector::Dot(&result_12, &out_pResult_12);
		CVector::Multiply((CVector*)&msg_12, &out_pResult_12, v81);
		v103 = vector_subtract((CVector)result_12, msg_12);
		//v25 = CVector::Scale(v103, v103);
		//if (v25.norm() > 1.0)
		if (v103.squaredNorm() > 1.0)
			v101 = *CVector::Normalized((CVector*)&v103, (CVector*)&v94);
		v26 = &This->m_2DPathFwd;
		if (CVector::Dot(&v101, &This->m_2DPathFwd) <= 0.0)
		{
			v80.x() = -v26->x();
			v80.y() = -This->m_2DPathFwd.y();
			v80.z() = -This->m_2DPathFwd.z();
			v27 = v80;
		}
		else
		{
			v27 = *v26;
		}
		v28 = This->m_pParams;
		v101 = v27;
		if (v28->IsPositionBasePlayer)
		{
			v74 = CVector::Dot(&result_12, &v101);
			if (v74 >= 0.0)
				v29 = v74;
			else
				v29 = 0.0;
			v30 = This->m_pParams;
			v31 = v30->TargetFrontOffsetSpeedScale * v29 + v30->TargetFrontOffset;
			if (v30->TargetFrontOffsetMax <= (double)v31)
				v31 = v30->TargetFrontOffsetMax;
			v69 = v91.get()->m_Offset + v31;
			CVector::Multiply((CVector*)&msg_12, &v101, v69);
			v32 = msg_12;
			v6 = !v91.get()->m_boolA;
			v83 = msg_12;
			if (!v6)
			{
				v80 = -v83;
				v32 = v80;
			}
			v33 = s_GlobalParam.TargetFrontOffsetSensitive * v86;
			if (v33 > 1.0)
				v33 = 1.0;
			msg_12 = vector_subtract((CVector)v32, (CVector)This->m_TargetFrontOffset);
			CVector::Multiply(&v80, (CVector*)&msg_12, v33);
			This->m_TargetFrontOffset = vector_add((CVector)This->m_TargetFrontOffset, (CVector)v80);
			lineNumber_12 = vector_add((CVector)This->m_TargetFrontOffset, (CVector)lineNumber_12);
		}
		if (This->m_pParams->IsPositionBasePlayer)
		{
			v34 = (float*)v89.get();
			if (*((char*)v89.get() + 36))
				This->m_TargetUpGroundDistance = This->m_Velocity.y() * *(float*)&a2 + This->m_TargetUpGroundDistance;
			msg_12 = (CVector)lineNumber_12;
			v35 = v34[8];
			if (v35 > (double)s_GlobalParam.TargetUpMaxGroundDistance)
				v35 = s_GlobalParam.TargetUpMaxGroundDistance;
			v37 = s_GlobalParam.SlopeSensitiveAir * v86;
			if (v37 > 1.0)
				v37 = 1.0;
			v36 = msg_12.y() - v35 * s_GlobalParam.TargetUpMoveRate;
			This->m_TargetUpGroundDistance = (v36 - This->m_TargetUpGroundDistance) * v37 + This->m_TargetUpGroundDistance;

			CMatrix _myMatrixA;
			CMatrix44 _myMatrixB;

			_myMatrixA = pCamera->m_MyCamera.m_View;

			MatMultiply(
				(CMatrix44*)&This_12,
				&pCamera->m_MyCamera.m_Projection,
				(CMatrix*)&_myMatrixA);

			_myMatrixB = This_12;

			msg_12 = MakeMatrix44Vector(_myMatrixB, v100);

			v80.x() = 0.5;
			v80.y() = 0.5;
			v80.z() = 0.0;
			CVector::Multiply((CVector*)&result_4, (CVector*)&msg_12, 0.5);
			msg_12 = vector_add(result_4, (CVector)v80);
			v94 = pCamera->m_MyCamera.m_View.GetVectorFromRow(1);
			v38 = &v94;
			result_4 = vector_add(*v38, v100);

			v83 = MakeMatrix44Vector(_myMatrixB, result_4);

			v80.x() = 0.5;
			v80.y() = 0.5;
			v80.z() = 0.0;
			CVector::Multiply((CVector*)&result_4, (CVector*)&v83, 0.5);
			v39 = This->m_pParams;
			v83 = vector_add(result_4, (CVector)v80);
			v40 = v83.y() - msg_12.y();
			v41 = v39->TargetUpOffset * v40;
			v42 = (s_GlobalParam.SlopeSensitiveVelocityScaleOffset + v41) / v40;
			v43 = (s_GlobalParam.TargetUpMaxOffsetPositive - v41) / v40;
			v44 = This->m_TargetUpGroundDistance;
			v45 = v43;
			if (v44 - v100.y() <= v45)
			{
				if (v100.y() - v44 <= v42)
				{
				LABEL_57:
					lineNumber_12.y() = This->m_TargetUpGroundDistance;
					goto LABEL_58;
				}
				v46 = v100.y() - v42;
			}
			else
			{
				v46 = v100.y() + v45;
			}
			This->m_TargetUpGroundDistance = v46;
			goto LABEL_57;
		}
	LABEL_58:
		if (*((char*)v89.get() + 36))
		{
			actorIDb = CVector::Length(&result_12) - s_GlobalParam.TargetUpOffsetSensitive;
			if (actorIDb >= 0.0)
				v47 = actorIDb;
			else
				v47 = 0.0;
			v48 = v47 * s_GlobalParam.SlopeSensitiveMax + s_GlobalParam.TargetUpMaxOffsetNegative;
			if (v48 <= (double)s_GlobalParam.SphericalPositionSensitiveY)
				actorIDa = v48;
			else
				actorIDa = s_GlobalParam.SphericalPositionSensitiveY;
		}
		else
		{
			actorIDa = s_GlobalParam.SlopeSensitiveVelocityScale;
		}
		v80.x() = 1.0;
		v80.y() = 0.0;
		v80.z() = 0.0;
		v83.x() = 0;
		v83.y() = 1;
		v83.z() = 0;
		CQuaternion::FromAxes(
			(CQuaternion*)&result_4,
			(CVector*)&v83,
			&out_pResult_12,
			&v80);
		v80.x() = 1.0;
		v80.y() = 0.0;
		v80.z() = 0.0;
		v83.x() = 0;
		v83.y() = 1;
		v83.z() = 0;
		CQuaternion::FromAxes(
			(CQuaternion*)&msg_12,
			(CVector*)&v83,
			&This->m_Field180,
			&v80);
		v49 = actorIDa * *(float*)&a2;
		if (v49 > 1.0)
			v49 = 1.0;
		*(CQuaternion*)&v94 = CQuaternion::Slerp(
			*(CQuaternion*)&msg_12,
			*(CQuaternion*)&result_4,
			v49);
		v80.x() = 0.0;
		v80.y() = 1.0;
		v80.z() = 0.0;

		This_12 = (*(CQuaternion*)&v94).ToRotationMatrix();
		v50 = &This_12;

		This->m_Field180 = *CMatrix::TransformNormal(v50, (CVector*)&v83, &v80);
		CVector::Multiply(&v80, &This->m_Field180, This->m_pParams->TargetUpOffset);
		v51 = This->m_pParams;
		lineNumber_12 = vector_add((CVector)v80, (CVector)lineNumber_12);
		v83 = (CVector)v51->SphericalPosition;
		v52 = a1_4.x() * a1_4.x();
		v53 = a1_4.z() * a1_4.z();
		v70 = v52 + v53;
		v81 = std::sqrt(v70);
		if (v81 >= 0.0099999998)
		{
			*(double*)&v80.x() = v83.z();
			v54 = atan2(a1_4.y(), v81);
			v83.z() = *(double*)&v80.x() - v54 * s_GlobalParam.SlopeRollRate;
		}
		else
		{
			v83.z() = v83.z() - s_GlobalParam.SlopeRollRate * 1.570796370506287;
		}
		v55 = atan2(a1_4.x(), a1_4.z());
		v83.y() = v55 + v83.y();
		AngleWrap(&v83.y(), This->m_AngularPosition.y());
		AngleWrap(&v83.z(), This->m_AngularPosition.z());
		v56 = v86;
		This->m_AngularPosition.x() = v83.x();
		v57 = s_GlobalParam.PlayerPositionSensitive * v56;
		if (v57 > 1.0)
			v57 = 1.0;
		v58 = (v83.y() - This->m_AngularPosition.y()) * v57 + This->m_AngularPosition.y();
		This->m_AngularPosition.y() = v58;
		v59 = s_GlobalParam.SlopeSensitive * v56;
		if (v59 > 1.0)
			v59 = 1.0;
		v60 = (v83.z() - This->m_AngularPosition.z()) * v59 + This->m_AngularPosition.z();
		This->m_AngularPosition.z() = v60;
		v61 = v58;
		if (v61 > M_PI)
		{
			v62 = v61 - M_PI * 2.0;
		LABEL_79:
			This->m_AngularPosition.y() = v62;
			goto LABEL_80;
		}
		if ((float)-M_PI > v61)
		{
			v62 = M_PI * 2.0 + This->m_AngularPosition.y();
			goto LABEL_79;
		}
	LABEL_80:
		v94 = ConvertToSpherePolar(This->m_AngularPosition);
		v102 = v94;
		//v102 = *(CVector*)ConvertToSpherePolar((CVector*)&v94, &This->m_AngularPosition);
		v63 = (CVector*)CVector::Normalized(
			(CVector*)&v102,
			(CVector*)&v94);
		v71 = CVector::Dot(v63, &pBaseSpacePathPosition);
		actorIDc = fabs(v71);
		if (actorIDc <= 0.949999988079071)
		{
			v65 = (CVector)pBaseSpacePathPosition;
		}
		else
		{
			v64 = (actorIDc - 0.949999988079071) / 0.05000000074505806;
			v81 = v64;
			v72 = 1.0 - v64;
			CVector::Multiply((CVector*)&result_4, &pBaseSpacePathPosition, v72);
			CVector::Multiply((CVector*)&v94, &This->m_Field180, v81);
			v65 = vector_add(v94, result_4);
			pBaseSpacePathPosition = v65;
		}
		v66 = lineNumber_12;
		This->m_CameraPosition1 = vector_add(v102, (CVector)lineNumber_12);
		v67 = v100;
		This->m_UpVector1 = v65;
		This->m_TargetPosition1 = v66;
		This->m_CameraPosition2 = vector_add((CVector)v67, a1_4);
		This->m_UpVector2 = out_pResult_12;
		This->m_TargetPosition2 = v67;
	}

#undef vector_subtract
#undef vector_add
#undef _mm_mul_ps

	void Camera2D_Cleaned(Sonic::CPlayer2DNormalCamera* This)
	{
		using namespace Hedgehog::Math;
		using namespace Sonic::Message;
		using namespace Sonic;
		
		auto MakeMatrix44Vector = [](const CMatrix44& in_rMatrix, const CVector& in_rBaseVector) -> CVector
		{
			FUNCTION_PTR(void, __thiscall, UnknownVectorMath, 0x006F2180, const CMatrix44 & _in_rMatrix, CVector * out_pVector, const CVector & _in_rBaseVector);
			CVector out = {};
			UnknownVectorMath(in_rMatrix, &out, in_rBaseVector);
			return out;
		};

		//float PlayerVelocitySensitive; // xmm0_4
		CVector* v17; // esi
		CVector* v19; // edi
		CVector* v20; // edi
		double v21; // xmm1_8
		float v22; // xmm0_4
		CVector v23; // xmm0
		CPlayer2DNormalCamera::SParams* v24; // eax
		CVector* v26; // esi
		CVector v27; // xmm0
		CPlayer2DNormalCamera::SParams* v28; // edx
		float v29; // xmm0_4
		CPlayer2DNormalCamera::SParams* v30; // eax
		float v31; // xmm0_4
		CVector v32; // xmm0
		float v33; // xmm1_4
		float v35; // xmm0_4
		float v36; // xmm2_4
		float v37; // xmm1_4
		CVector* v38; // eax
		CPlayer2DNormalCamera::SParams* v39; // edx
		float v40; // xmm1_4
		double v41; // xmm0_8
		float v42; // xmm3_4
		double v43; // xmm2_8
		float v44; // xmm0_4
		float v45; // xmm1_4
		double v46; // xmm0_8
		float v47; // xmm1_4
		float v48; // xmm1_4
		float v49; // xmm0_4
		CMatrix44* v50; // eax
		CPlayer2DNormalCamera::SParams* v51; // eax
		float v52; // xmm1_4
		float v53; // xmm0_4
		double v54; // st7
		double v55; // st7
		float v57; // xmm1_4
		const CVector* v68; // [esp+0h] [ebp-218h]
		float v69; // [esp+0h] [ebp-218h]
		float v70; // [esp+0h] [ebp-218h]
		float v74; // [esp+18h] [ebp-200h]
		float actorIDa; // [esp+1Ch] [ebp-1FCh]
		float actorIDb; // [esp+1Ch] [ebp-1FCh]
		float SphericalAngle; // [esp+1Ch] [ebp-1FCh]
		CVector v80; // [esp+24h] [ebp-1F4h] BYREF
		CVector v83; // [esp+44h] [ebp-1D4h] BYREF
		boost::shared_ptr<MsgGetGroundInfo> v89; // [esp+8Ch] [ebp-18Ch] BYREF
		boost::shared_ptr<MsgGetCameraTargetPosition> v91; // [esp+ACh] [ebp-16Ch] BYREF
		CVector v103; // [esp+144h] [ebp-D4h] BYREF
		
		

		CCamera* pCamera = This->GetContext();
		int actorID = pCamera->m_ActorID;
		const float deltaTime = This->GetDeltaTime();
		float frameDeltaTime = deltaTime * 60.0;

		const struct
		{
			CVector targetPosition;
			CVector velocity;
			CVector frontDirection;
		}playerInfo;

		struct
		{
			const float distance;
			const CVector up;
			const CVector fwd;
		}pathInfo = {};
		

		boost::shared_ptr<MsgGetGroundInfo> spGroundInfo               = boost::make_shared<MsgGetGroundInfo>();
		boost::shared_ptr<MsgGetCameraTargetPosition> spTargetPosition = boost::make_shared<MsgGetCameraTargetPosition>(const_cast<CVector*>(&playerInfo.targetPosition));

		boost::shared_ptr<MsgGetForCamera2DState> msgForCam2DState = boost::make_shared<MsgGetForCamera2DState>();
		boost::shared_ptr<MsgGet2DPathPNT> spMsgPathPNT =
			boost::make_shared<MsgGet2DPathPNT>(const_cast<float*>(&pathInfo.distance),
			                                    &This->m_PointPosition,
			                                    const_cast<CVector*>(&pathInfo.up), 
			                                    const_cast<CVector*>(&pathInfo.fwd));


		spTargetPosition->m_Offset = 0.0f;
		spTargetPosition->m_boolA = false;
		spTargetPosition->m_NoOffset = true;
		spTargetPosition->m_UseModelMatrix = true;


		pCamera->SendMessageImm(actorID, spTargetPosition);
		pCamera->SendMessageImm(actorID, boost::make_shared<MsgGetVelocity>(const_cast<CVector*>(&playerInfo.velocity)));
		pCamera->SendMessageImm(actorID, boost::make_shared<MsgGetFrontDirection>(const_cast<CVector*>(&playerInfo.frontDirection)));
		pCamera->SendMessageImm(actorID, spGroundInfo);

		const CVector TargetUpGroundDistance = spGroundInfo->m_Distance <= (double)s_GlobalParam.TargetUpMaxGroundDistance
		                                     ? spGroundInfo->m_GroundNormal : CVector(0, 1, 0);

		const Eigen::AlignedVector3<float> a = {};
		const CVector b = a;

		CVector cross;
		bool negateDirection = false;
		if (!This->m_pParams->IsBaseSpacePlayer)
		{
			cross = CVector::Cross(This->m_2DPathFwd, This->m_2DPathUp);
		}
		else
		{
			if (!pCamera->SendMessageImm(actorID, spMsgPathPNT))
			{
				This->m_CameraPosition1 = pCamera->m_Position;
				This->m_UpVector1 = CVector::Up();
				This->m_TargetPosition1 = pCamera->m_TargetPosition;
				This->m_CameraPosition2 = pCamera->m_Position;
				This->m_UpVector2 = CVector::Up();
				This->m_TargetPosition2 = pCamera->m_TargetPosition;
				return;
			}

			if (pCamera->SendMessageImm(actorID, msgForCam2DState))
				negateDirection = msgForCam2DState->m_IsValid;

			This->m_pParams->BaseSpacePathPosition = pathInfo.distance;

			// Eigen template jank is causing this to scream about conversions, so just make temp variables.
			const float negation = CVector::Dot(pathInfo.up, This->m_2DPathUp) <= -0.25 || CVector::Dot(pathInfo.fwd, This->m_2DPathFwd) >= -0.25
			                     ? 1.0f : -1.0f;

			This->m_2DPathFwd = pathInfo.fwd * negation;
			This->m_2DPathUp  = pathInfo.up;

			cross = CVector::Cross(This->m_2DPathFwd, negateDirection ? -This->m_2DPathUp : This->m_2DPathUp);
		}

		const float PlayerVelocitySensitive = fminf(s_GlobalParam.PlayerVelocitySensitive * frameDeltaTime, 1.0f);
		v17 = &This->m_Velocity;

		CVector _pathUp = playerInfo.velocity - This->m_Velocity;
		v80 = _pathUp * PlayerVelocitySensitive;
		This->m_Velocity = This->m_Velocity + v80;
		if (spGroundInfo->m_OnGround)
		{
			v19 = &spGroundInfo->m_GroundNormal;
			_pathUp = *v19 * CVector::Dot(This->m_Velocity, spGroundInfo->m_GroundNormal);
			v20 = &spGroundInfo->m_GroundNormal;
			v68 = &spGroundInfo->m_GroundNormal;
			*v17 = *v17 - _pathUp;
			_pathUp = *v20 * CVector::Dot(playerInfo.velocity, *v68);
			*v17 = *v17 + _pathUp;
		}

		const CVector _velocity = *v17;
		_pathUp = _velocity * deltaTime;
		v21 = frameDeltaTime;
		This->m_SphericalPosition = This->m_SphericalPosition + _pathUp;
		v22 = s_GlobalParam.SphericalPositionSensitiveZ * v21;
		if (v22 > 1.0)
			v22 = 1.0;
		_pathUp = playerInfo.targetPosition - This->m_SphericalPosition;
		v80 = _pathUp * v22;
		This->m_SphericalPosition = This->m_SphericalPosition + v80;
		v23 = This->m_SphericalPosition;
		v24 = This->m_pParams;

		const CVector _sphericalPosition = v23;
		if (!v24->IsPositionBasePlayer)
			v23 = This->m_PointPosition;

		// This will be written to and modified as we go.
		CVector CameraPosition = v23;

		_pathUp = TargetUpGroundDistance * CVector::Dot(_velocity, TargetUpGroundDistance);
		v103 = _velocity - _pathUp;

		CVector _frontDirectionBase = v103.squaredNorm() > 1.0
		                            ? v103.normalizedSafe()
		                            : playerInfo.frontDirection;
		v26 = &This->m_2DPathFwd;
		if (CVector::Dot(_frontDirectionBase, This->m_2DPathFwd) <= 0.0)
		{
			v80.x() = -v26->x();
			v80.y() = -This->m_2DPathFwd.y();
			v80.z() = -This->m_2DPathFwd.z();
			v27 = v80;
		}
		else
		{
			v27 = *v26;
		}
		v28 = This->m_pParams;

		CVector _frontDirection = v27;
		if (v28->IsPositionBasePlayer)
		{
			v74 = CVector::Dot(_velocity, _frontDirection);
			if (v74 >= 0.0)
				v29 = v74;
			else
				v29 = 0.0;
			v30 = This->m_pParams;
			v31 = v30->TargetFrontOffsetSpeedScale * v29 + v30->TargetFrontOffset;
			if (v30->TargetFrontOffsetMax <= (double)v31)
				v31 = v30->TargetFrontOffsetMax;
			v69 = spTargetPosition.get()->m_Offset + v31;
			_pathUp = _frontDirection * v69;
			v32 = _pathUp;
			bool unknownBool = !spTargetPosition.get()->m_boolA;
			v83 = _pathUp;
			if (!unknownBool)
			{
				v80 = -v83;
				v32 = v80;
			}
			v33 = s_GlobalParam.TargetFrontOffsetSensitive * frameDeltaTime;
			if (v33 > 1.0)
				v33 = 1.0;
			_pathUp = v32 - This->m_TargetFrontOffset;
			This->m_TargetFrontOffset += (_pathUp * v33);
			CameraPosition += This->m_TargetFrontOffset;
		}
		if (This->m_pParams->IsPositionBasePlayer)
		{
			if (spGroundInfo->m_OnGround)
				This->m_TargetUpGroundDistance = This->m_Velocity.y() * deltaTime + This->m_TargetUpGroundDistance;
			_pathUp = CameraPosition;
			v35 = spGroundInfo->m_Distance;
			if (v35 > (double)s_GlobalParam.TargetUpMaxGroundDistance)
				v35 = s_GlobalParam.TargetUpMaxGroundDistance;
			v37 = s_GlobalParam.SlopeSensitiveAir * frameDeltaTime;
			if (v37 > 1.0)
				v37 = 1.0;
			v36 = _pathUp.y() - v35 * s_GlobalParam.TargetUpMoveRate;
			This->m_TargetUpGroundDistance = (v36 - This->m_TargetUpGroundDistance) * v37 + This->m_TargetUpGroundDistance;
			
			CMatrix44 projectedViewMatrix = pCamera->m_MyCamera.m_Projection * pCamera->m_MyCamera.m_View;
			_pathUp = MakeMatrix44Vector(projectedViewMatrix, _sphericalPosition);

			v80.x() = 0.5;
			v80.y() = 0.5;
			v80.z() = 0.0;
			CVector vectorA = _pathUp * 0.5;
			_pathUp = vectorA + v80;
			CVector viewVector = pCamera->m_MyCamera.m_View.GetVectorFromRow(1);
			v38 = &viewVector;
			vectorA = *v38 + _sphericalPosition;

			v83 = MakeMatrix44Vector(projectedViewMatrix, vectorA);

			v80.x() = 0.5;
			v80.y() = 0.5;
			v80.z() = 0.0;
			vectorA = v83 * 0.5;
			v39 = This->m_pParams;
			v83 = vectorA + v80;
			v40 = v83.y() - _pathUp.y();
			v41 = v39->TargetUpOffset * v40;
			v42 = (s_GlobalParam.SlopeSensitiveVelocityScaleOffset + v41) / v40;
			v43 = (s_GlobalParam.TargetUpMaxOffsetPositive - v41) / v40;
			v44 = This->m_TargetUpGroundDistance;
			v45 = v43;

			if (v44 - _sphericalPosition.y() <= v45)
			{
				if (_sphericalPosition.y() - v44 <= v42)
				{
					CameraPosition.y() = This->m_TargetUpGroundDistance;
				}
				else
				{
					v46 = _sphericalPosition.y() - v42;
					This->m_TargetUpGroundDistance = v46;
					CameraPosition.y() = This->m_TargetUpGroundDistance;
				}
			}
			else
			{
				v46 = _sphericalPosition.y() + v45;
				This->m_TargetUpGroundDistance = v46;
				CameraPosition.y() = This->m_TargetUpGroundDistance;
			}
		}

		if (*((char*)spGroundInfo.get() + 36))
		{
			actorIDb = _velocity.norm() - s_GlobalParam.TargetUpOffsetSensitive;
			if (actorIDb >= 0.0)
				v47 = actorIDb;
			else
				v47 = 0.0;
			v48 = v47 * s_GlobalParam.SlopeSensitiveMax + s_GlobalParam.TargetUpMaxOffsetNegative;
			if (v48 <= (double)s_GlobalParam.SphericalPositionSensitiveY)
				actorIDa = v48;
			else
				actorIDa = s_GlobalParam.SphericalPositionSensitiveY;
		}
		else
		{
			actorIDa = s_GlobalParam.SlopeSensitiveVelocityScale;
		}
		v80.x() = 1.0;
		v80.y() = 0.0;
		v80.z() = 0.0;
		v83.x() = 0;
		v83.y() = 1;
		v83.z() = 0;
		CQuaternion result_4 = CQuaternion::FromAxes(v83, TargetUpGroundDistance, v80);
		v80.x() = 1.0;
		v80.y() = 0.0;
		v80.z() = 0.0;
		v83.x() = 0;
		v83.y() = 1;
		v83.z() = 0;
		CQuaternion::FromAxes(
			(CQuaternion*)&_pathUp,
			&v83,
			&This->m_Field180,
			&v80);
		v49 = actorIDa * deltaTime;
		if (v49 > 1.0)
			v49 = 1.0;
		CQuaternion v94 = CQuaternion::Slerp(
			*(CQuaternion*)&_pathUp,
			*(CQuaternion*)&result_4,
			v49);
		v80.x() = 0.0;
		v80.y() = 1.0;
		v80.z() = 0.0;

		CMatrix44 This_12 = (*(CQuaternion*)&v94).ToRotationMatrix();
		v50 = &This_12;

		This->m_Field180 = *CMatrix::TransformNormal(v50, &v83, &v80);
		CameraPosition = This->m_Field180 * This->m_pParams->TargetUpOffset + CameraPosition;

		float sqrMag = std::sqrt((cross.x() * cross.x()) + (cross.z() * cross.z()));
		if (sqrMag >= 0.0099999998)
		{
			const double sphereX = This->m_pParams->SphericalPosition.z();
			This->m_pParams->SphericalPosition.z() = sphereX - atan2(cross.y(), sqrMag) * s_GlobalParam.SlopeRollRate;
		}
		else
		{
			This->m_pParams->SphericalPosition.z() = This->m_pParams->SphericalPosition.z() - s_GlobalParam.SlopeRollRate * 1.570796370506287;
		}

		This->m_pParams->SphericalPosition.y() = atan2(cross.x(), cross.z()) + This->m_pParams->SphericalPosition.y();
		AngleWrap(&This->m_pParams->SphericalPosition.y(), This->m_AngularPosition.y());
		AngleWrap(&This->m_pParams->SphericalPosition.z(), This->m_AngularPosition.z());

		This->m_AngularPosition.x() = This->m_pParams->SphericalPosition.x();
		This->m_AngularPosition.y() += (This->m_pParams->SphericalPosition.y() - This->m_AngularPosition.y())
		                             * fminf(s_GlobalParam.PlayerPositionSensitive * frameDeltaTime, 1.0f);
		This->m_AngularPosition.z() += (This->m_pParams->SphericalPosition.z() - This->m_AngularPosition.z())
		                             * fminf(s_GlobalParam.SlopeSensitive * frameDeltaTime, 1.0f);

		if (This->m_AngularPosition.y() > M_PI)
		{
			This->m_AngularPosition.y() -= M_PI * 2.0;
		}
		else if (This->m_AngularPosition.y() < -M_PI)
		{
			This->m_AngularPosition.y() += M_PI * 2.0;
		}

		const CVector SphericalPosition = ConvertToSpherePolar(This->m_AngularPosition);
		SphericalAngle = fabs(CVector::Dot(SphericalPosition.normalizedSafe(), CVector(0, 1, 0)));

		CVector CameraUpVector = SphericalAngle > 0.95
		                       ? This->m_Field180 * ((SphericalAngle - 0.95) / 0.05) + CVector(0, 1, 0) * (1.0 - (SphericalAngle - 0.95) / 0.05)
		                       : CVector(0, 1, 0);

		This->m_CameraPosition1 = SphericalPosition + CameraPosition;
		This->m_UpVector1 = CameraUpVector;
		This->m_TargetPosition1 = CameraPosition;
		This->m_CameraPosition2 = _sphericalPosition + cross;
		This->m_UpVector2 = TargetUpGroundDistance;
		This->m_TargetPosition2 = _sphericalPosition;
	}

	static int useIndex = 1;

	HOOK(void, __fastcall, _Camera2DNormalUpdate, 0x010F3BA0, Sonic::CPlayer2DNormalCamera* This)
	{
		DebugDrawText::log("Camera2D");
		switch (useIndex)
		{
		case 1:
		default:
			original_Camera2DNormalUpdate(This);
			break;
		case 2:
			Camera2D_GroundTruth(This);
			break;
		case 3:
			Camera2D_Cleaned(This);
			break;
		}
	}

}


extern "C" __declspec(dllexport) void Init()
{
	using namespace Mod;

	MessageBoxA(nullptr, "Begin!", "", MB_OK);
	INSTALL_HOOK(_MainCollisionFunc)
		INSTALL_HOOK(_Camera2DNormalUpdate)
		INSTALL_HOOK(InitializeApplicationParams)
}

extern "C" __declspec(dllexport) void OnFrame()
{
	using namespace Mod;

	if (GetAsyncKeyState('1') & 1)
		useIndex = 1;
	if (GetAsyncKeyState('2') & 1)
		useIndex = 2;
	if (GetAsyncKeyState('3') & 1)
		useIndex = 3;

	DebugDrawText::log(format("Camera State - %d", useIndex));
}
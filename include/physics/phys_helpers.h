#pragma once

#include "math/Vec3.h"

namespace physics
{

// Origin space is user design space, i.e. the space where point 0 0 0 maps to the
//	point in the world space, where body was initially placed (i.e. not the center
//	of mass space)

// Transforms vector from world space to the space relative to the body origin;
static math::Vec3 transformWorldToOrigin(const NodeTranslational * bodyL, const NodeRotational * bodyR, const math::Vec3 & pointVsWorld)
{
	if (!bodyL)
		return pointVsWorld;

	if (!bodyR)
		return pointVsWorld - bodyL->m_pos;

	math::Vec3 originPos = bodyL->m_pos + bodyR->m_rot.rotate(-bodyR->m_com);
	return bodyR->m_rot.getConjugated().rotate(pointVsWorld - originPos);
}

static math::Vec3 transformOriginToWorld(const NodeTranslational * bodyL, const NodeRotational * bodyR, const math::Vec3 & pointVsOrigin)
{
	if (!bodyL)
		return pointVsOrigin;

	if (!bodyR)
		return bodyL->m_pos + pointVsOrigin;

	return bodyL->m_pos + bodyR->m_rot.rotate(pointVsOrigin - bodyR->m_com);
}

}

#pragma once

#define WIN32_LEAN_AND_MEAN
#define _SILENCE_ALL_CXX23_DEPRECATION_WARNINGS
#define _USE_LOG
//#define NOMINMAX

//constexpr float RAD2DEGf = 57.2958f;
//constexpr float DEG2RADf = 0.0174533f;

//constexpr double RAD2DEG = 57.29578018188477;
//constexpr double DEG2RAD = 0.01745329238474369;

//constexpr float  RAD2REVOLUTIONf = RAD2DEGf / 360.0f;
//constexpr double RAD2REVOLUTION = RAD2DEG / 360.0;

#define BB_EXCLUDE_MATH_DEFINITIONS
#include <BlueBlur.h>

#include <Windows.h>
#include <detours.h>

#include <cstdint>
#include <cstdio>

#include <INIReader.h>

#include <Helpers.h>

template <typename T = uint32_t*, typename A>
T GetItem(A* ptr, uint32_t offset)
{
	return *(T*)((uint32_t)ptr + offset);
}

template <typename T = uint32_t*>
T GetItem(int ptr, uint32_t offset)
{
	return *(T*)((uint32_t)ptr + offset);
}

template <typename T = uint32_t*, typename A>
T* LGetItem(A* ptr, uint32_t offset)
{
	return (T*)((uint32_t)ptr + offset);
}
#pragma once

#define WIN32_LEAN_AND_MEAN

// Detours
#include <Windows.h>
#include "detours\include\detours.h"

// BlueBlur must be included before STL
#include <BlueBlur.h>

// Standard library
#include <cstdint>
#include <cstdio>
#include <string>

// Dependencies
#include "ArchiveTreePatcher.h"
#include "include\Common.h"
#include "Configuration.h"
#include "ControllerPatcher.h"
#include "DllMain.h"
#include "HudSonicStage.h"
#include "include\Helpers.h"
#include "include\INIReader.h"
#include "Patches.h"
#include "Resources.h"
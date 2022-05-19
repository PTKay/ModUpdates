#pragma once

#define INI_FILE "UnleashedHUD.ini"

class Configuration
{
public:
	static void Read();

	/// <summary>
	/// Determines whether the Unleashed window title will be used.
	/// </summary>
	static bool windowTitle;

	static enum class WindowTitleIcon : int
	{
		X360,
		X360_Preview,
		PS3
	};

	/// <summary>
	/// Determines what icon the Unleashed window title will use.
	/// </summary>
	static WindowTitleIcon windowTitleIcon;

	static enum WindowTitleRegion
	{
		English,
		Japanese
	};

	/// <summary>
	/// Determines what region the Unleashed window title will use.
	/// </summary>
	static WindowTitleRegion windowTitleRegion;

	static enum class ButtonType : byte
	{
		X360,
		XSX,
		PS3,
		Switch
	};

	/// <summary>
	/// Controlls what button prompts to show in-game.
	/// </summary>
	static ButtonType buttonType;
};


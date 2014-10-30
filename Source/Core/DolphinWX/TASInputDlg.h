// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2
// Refer to the license.txt file included.

#pragma once

#include <wx/bitmap.h>
#include <wx/dcmemory.h>
#include <wx/dialog.h>
#include <wx/event.h>
#include <wx/gdicmn.h>
#include <wx/sizer.h>
#include <wx/string.h>
#include <wx/toplevel.h>
#include <wx/translation.h>
#include <wx/windowid.h>

#include "Common/CommonTypes.h"
#include "Core/HW/WiimoteEmu/WiimoteEmu.h"
#include "InputCommon/GCPadStatus.h"

class wxCheckBox;
class wxSlider;
class wxStaticBitmap;
class wxTextCtrl;
class wxWindow;

class TASInputDlg : public wxDialog
{
	public:
		TASInputDlg(wxWindow* parent,
		            wxWindowID id = 1,
		            const wxString& title = _("TAS Input"),
		            const wxPoint& pos = wxDefaultPosition,
		            const wxSize& size = wxDefaultSize,
		            long style = wxDEFAULT_DIALOG_STYLE | wxSTAY_ON_TOP);

		void OnCloseWindow(wxCloseEvent& event);
		void UpdateFromSliders(wxCommandEvent& event);
		void UpdateFromText(wxCommandEvent& event);
		void OnMouseDownL(wxMouseEvent& event);
		void OnMouseUpR(wxMouseEvent& event);
		void OnRightClickSlider(wxMouseEvent& event);
		void ResetValues();
		void GetValues(GCPadStatus* PadStatus);
		void GetValues(u8* data, WiimoteEmu::ReportFeatures rptf);
		void SetTurbo(wxMouseEvent& event);
		void SetTurboFalse(wxMouseEvent& event);
		void SetTurboState(wxCheckBox* CheckBox, bool* TurboOn);
		void ButtonTurbo();
		void GetKeyBoardInput(GCPadStatus* PadStatus);
		void GetKeyBoardInput(u8* data, WiimoteEmu::ReportFeatures rptf);
		bool TextBoxHasFocus();
		void SetLandRTriggers();
		bool TASHasFocus();
		void CreateGCLayout();
		void CreateWiiLayout();
		wxBitmap CreateStickBitmap(int x, int y);
		void SetWiiButtons(u16* butt);
		void GetIRData(u8* const data, u8 mode, bool use_accel);

	private:
		const int ID_C_STICK = 1001;
		const int ID_MAIN_STICK = 1002;
		int m_eleID = 1003;

		struct Control
		{
			wxTextCtrl* text;
			wxSlider* slider;
			int value = -1;
			int text_id;
			int slider_id;
			u32 range;
			u32 default_value = 128;
			bool set_by_keyboard = false;
		};

		struct Button
		{
			wxCheckBox* checkbox;
			bool set_by_keyboard = false;
			bool turbo_on = false;
			int id;
		};

		struct Stick
		{
			wxStaticBitmap* bitmap;
			Control x_cont;
			Control y_cont;
		};

		void SetStickValue(bool* ActivatedByKeyboard, int* AmountPressed, wxTextCtrl* Textbox, int CurrentValue, int center = 128);
		void SetButtonValue(Button* button, bool CurrentState);
		void SetSliderValue(Control* control, int CurrentValue, int defaultValue = 128);
		Stick CreateStick(int id_stick);
		wxStaticBoxSizer* CreateStickLayout(Stick* tempStick, const wxString& title);
		Button CreateButton(const std::string& name);
		Control CreateControl(long style, int width, int height, u32 range = 255);

		Control m_l_cont, m_r_cont, m_x_cont, m_y_cont, m_z_cont;
		Button m_a, m_b, m_x, m_y, m_z, m_l, m_r, m_c;
		Button m_start, m_plus, m_minus, m_one, m_two, m_home;
		Button m_dpad_up, m_dpad_down, m_dpad_left, m_dpad_right;
		Stick m_main_stick, m_c_stick;

		Button* m_buttons[14];
		Control* m_controls[10];
		static const int m_gc_pad_buttons_bitmask[12];
		static const int m_wii_buttons_bitmask[13];

		bool m_has_layout = false;
		bool m_is_wii = false;

		wxGridSizer* const m_buttons_dpad = new wxGridSizer(3);

		DECLARE_EVENT_TABLE();
};

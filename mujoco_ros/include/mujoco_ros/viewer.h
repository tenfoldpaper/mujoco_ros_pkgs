#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <ratio>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mjui.h>
#include <mujoco/mujoco.h>

#include <mujoco_ros/array_safety.h>

#include <mujoco_ros/platform_ui_adapter.h>
#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_env.h>

namespace mujoco_ros {

// The viewer itself doesn't require a reentrant mutex, however we use it in
// order to provide a Python sync API that doesn't require separate locking
// (since sync is by far the most common operation), but that also won't
// deadlock if called when a lock is already held by the user script on the
// same thread.
class ViewerMutex : public std::recursive_mutex
{};
using MutexLock = std::unique_lock<std::recursive_mutex>;

class Viewer
{
public:
	using Clock = std::chrono::steady_clock;
	static_assert(std::ratio_less_equal_v<Clock::period, std::milli>, "Clock must have millisecond precision or better");

	static int constexpr kMaxGeom = 20000;

	// Create object and initialize the ui
	Viewer(std::unique_ptr<PlatformUIAdapter> platform_ui_adapter, MujocoEnv *env, bool fully_managed);

	// Apply UI pose perturbations to model and data
	void ApplyForcePerturbations(int flg_paused);

	// Apply UI pose perturbations to model and data
	void ApplyForcePerturbations();

	// Syncronize mjModel and mjData state with UI inputs, and update visualization
	void Sync();

	// Request that the simulation UI thread renders a new model optionally deleting the old model and data when done
	void Load(mjModelPtr m, mjDataPtr d, const char *displayed_filename);

	// functions below are used by the render thread

	// load mjb or xml model that has been requested by load()
	void LoadOnRenderThread();

	// render the UI to the window
	void Render();

	// loop to render the UI
	void RenderLoop();

	// constants
	static constexpr int kMaxFilenameLength             = 1000;
	static constexpr double render_ui_rate_lower_bound_ = 0.0333; // Minimum render freq at 30 fps
	static constexpr float render_ui_rate_upper_bound_  = 0.0166f; // Maximum render freq at 60 fps

	// whether the viewer is operating in fully managed mode, where it can assume
	// that it has exclusive access to the model, data, and various mjv objects
	bool fully_managed_ = true;

	// Reference to env
	MujocoEnv *env_;

	// model and data to be visualized
	mjModelPtr mnew_;
	mjDataPtr dnew_;

	mjModelPtr m_;
	mjDataPtr d_;

	int ncam_ = 0;
	int nkey_ = 0;

	std::vector<int> body_parentid_;

	std::vector<int> jnt_type_;
	std::vector<int> jnt_group_;
	std::vector<int> jnt_qposadr_;
	std::vector<std::optional<std::pair<mjtNum, mjtNum>>> jnt_range_;
	std::vector<std::string> jnt_names_;

	std::vector<int> actuator_group_;
	std::vector<std::optional<std::pair<mjtNum, mjtNum>>> actuator_ctrlrange_;
	std::vector<std::string> actuator_names_;

	// mjModel and mjData fields that can be modified by the user through the GUI
	std::vector<mjtNum> qpos_;
	std::vector<mjtNum> qpos_prev_;
	std::vector<mjtNum> ctrl_;
	std::vector<mjtNum> ctrl_prev_;

	mjvSceneState scnstate_;
	mjOption mjopt_prev_;
	mjvOption opt_prev_;
	mjvCamera cam_prev_;
	int warn_vgeomfull_prev_;

	// pending GUI-driven actions, to be applied at the next call to Sync
	struct
	{
		std::optional<std::string> save_xml;
		std::optional<std::string> save_mjb;
		std::optional<std::string> print_model;
		std::optional<std::string> print_data;
		bool align;
		bool copy_pose;
		bool load_key;
		bool save_key;
		bool zero_ctrl;
		int newperturb;
		bool select;
		mjuiState select_state;
		bool ui_update_physics;
		bool ui_update_rendering;
		bool ui_update_joint;
		bool ui_update_ctrl;

		// UBI Extra
		bool ui_update_run   = false;
		bool ui_exit         = false;
		bool ui_update_speed = false;
		bool ui_reset        = false;
		bool ui_reload       = false;
	} pending_ = {};

	ViewerMutex mtx; // Should move to MujocoEnv
	std::condition_variable_any cond_loadrequest;

	int frames_ = 0;
	std::chrono::time_point<Clock> last_fps_update_;
	double fps_ = 0;

	// options
	int spacing    = 0;
	int color      = 0;
	int font       = 0;
	int ui0_enable = 0;
	int ui1_enable = 0;
	int help       = 0;
	int info       = 1;
	int profiler   = 0;
	int sensor     = 0;
	int fullscreen = 0;
	int vsync      = 1;
	int busywait   = 0;

	// keyframe index
	int key = 0;

	// UI proxy elements for Env settings
	int run = 0;

	// atomics for cross-thread messages
	std::atomic_int exit_request        = { 0 };
	std::atomic_int visual_init_request = { 0 };
	std::atomic_int ui_load_request     = { 0 };
	std::atomic_int dropload_request    = { 0 };
	std::atomic_int reset_request       = { 0 };
	std::atomic_int model_valid         = { false };
	std::atomic_int manual_env_steps    = { 0 };
	std::atomic_int screenshot_request  = { 0 };

	// load request
	//   0: model loaded or no load requested
	//   1: in the process of loading
	//   2: load requested from another thread
	std::atomic_int loadrequest = { 0 };

	// strings
	char load_error[kMaxFilenameLength]        = "";
	char filename[kMaxFilenameLength]          = "";
	char dropfilename[kMaxFilenameLength]      = "";
	char previous_filename[kMaxFilenameLength] = "";

	// time sync
	int real_time_index     = 1;
	float measured_slowdown = 1.0f;

	// control noise
	double ctrl_noise_std  = 0.0;
	double ctrl_noise_rate = 0.0;

	// watch
	char field[mjMAXUITEXT] = "qpos";
	int index               = 0;

	// physics: need sync
	int disable[mjNDISABLE] = { 0 };
	int enable[mjNENABLE]   = { 0 };

	// rendering: need sync
	int camera = 0;

	// visualization
	mjvScene &scn;
	mjvCamera cam;
	mjvOption opt;
	mjvPerturb &pert;
	mjvFigure figconstraint = {};
	mjvFigure figcost       = {};
	mjvFigure figtimer      = {};
	mjvFigure figsize       = {};
	mjvFigure figsensor     = {};

	// OpenGL rendering and UI
	int refresh_rate   = 60;
	int window_pos[2]  = { 0 };
	int window_size[2] = { 0 };
	std::unique_ptr<PlatformUIAdapter> platform_ui;
	mjuiState &uistate;
	mjUI ui0 = {};
	mjUI ui1 = {};

	// Constant arrays needed for the option section of UI and the UI interface
	// TODO setting the size here is not ideal
	const mjuiDef def_option[14] = { { mjITEM_SECTION, "Option", 1, nullptr, "AO" },
		                              { mjITEM_SELECT, "Spacing", 1, &this->spacing, "Tight\nWide" },
		                              { mjITEM_SELECT, "Color", 1, &this->color, "Default\nOrange\nWhite\nBlack" },
		                              { mjITEM_SELECT, "Font", 1, &this->font,
		                                "50 %\n100 %\n150 %\n200 %\n250 %\n300 %" },
		                              { mjITEM_CHECKINT, "Left UI (Tab)", 1, &this->ui0_enable, " #258" },
		                              { mjITEM_CHECKINT, "Right UI", 1, &this->ui1_enable, "S#258" },
		                              { mjITEM_CHECKINT, "Help", 2, &this->help, " #290" },
		                              { mjITEM_CHECKINT, "Info", 2, &this->info, " #291" },
		                              { mjITEM_CHECKINT, "Profiler", 2, &this->profiler, " #292" },
		                              { mjITEM_CHECKINT, "Sensor", 2, &this->sensor, " #293" },
		                              { mjITEM_CHECKINT, "Fullscreen", 1, &this->fullscreen, " #294" },
		                              { mjITEM_CHECKINT, "Vertical Sync", 1, &this->vsync, "" },
		                              { mjITEM_CHECKINT, "Busy Wait", 1, &this->busywait, "" },
		                              { mjITEM_END } };

	// simulation section of UI
	const mjuiDef def_simulation[12] = { { mjITEM_SECTION, "Simulation", 1, nullptr, "AS" },
		                                  { mjITEM_RADIO, "", 5, &this->pending_.ui_update_run, "Pause\nRun" },
		                                  { mjITEM_BUTTON, "Reset", 2, nullptr, " #259" },
		                                  { mjITEM_BUTTON, "Reload", 5, nullptr, "CL" },
		                                  { mjITEM_BUTTON, "Align", 2, nullptr, "CA" },
		                                  { mjITEM_BUTTON, "Copy pose", 2, nullptr, "CC" },
		                                  { mjITEM_SLIDERINT, "Key", 3, &this->key, "0 0" },
		                                  { mjITEM_BUTTON, "Load key", 3 },
		                                  { mjITEM_BUTTON, "Save key", 3 },
		                                  { mjITEM_SLIDERNUM, "Noise scale", 5, &this->ctrl_noise_std, "0 2" },
		                                  { mjITEM_SLIDERNUM, "Noise rate", 5, &this->ctrl_noise_rate, "0 2" },
		                                  { mjITEM_END } };

	// watch section of UI
	const mjuiDef def_watch[5] = { { mjITEM_SECTION, "Watch", 0, nullptr, "AW" },
		                            { mjITEM_EDITTXT, "Field", 2, this->field, "qpos" },
		                            { mjITEM_EDITINT, "Index", 2, &this->index, "1" },
		                            { mjITEM_STATIC, "Value", 2, nullptr, " " },
		                            { mjITEM_END } };

	// info strings
	char info_title[Viewer::kMaxFilenameLength]   = { 0 };
	char info_content[Viewer::kMaxFilenameLength] = { 0 };
};

} // namespace mujoco_ros

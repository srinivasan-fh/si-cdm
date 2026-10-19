#pragma once

// Launch splash: a borderless window showing a cinematic, real-time rendered
// Mars rover scene with the product title and the loading progress of the
// workbench. It owns its own GL and ImGui contexts and restores the caller's
// contexts after every frame, so the main window can be built while it shows.

#include <memory>
#include <string>
#include <vector>

struct GLFWwindow;
struct ImGuiContext;
struct ImFont;

namespace sicdm {

class Shader;

// Sets the window / taskbar icon from the embedded app icon (Windows, Linux;
// on macOS the bundle icon is used).
void applyWindowIcon(GLFWwindow* window);

class Splash {
public:
    // `minSeconds`: shortest time the splash stays up, so the scene and the
    // progress read calmly even when loading is instant.
    explicit Splash(float minSeconds = 3.2f);
    ~Splash();

    Splash(const Splash&) = delete;
    Splash& operator=(const Splash&) = delete;

    bool ok() const { return m_window != nullptr; }

    // Reports a loading step: the bar moves towards `progress` (0..1) and
    // `status` shows once the bar gets there. Draws a frame.
    void step(float progress, const char* status);

    // Plays the bar to 100 %, waits out the minimum time (a click or Esc skips),
    // fades out and closes the window.
    void finish();

private:
    void frame();
    void drawScene(int fbw, int fbh, float imageTop, float imageHeight, float fade);
    void drawOverlay(float fade);
    bool skipRequested() const;

    GLFWwindow* m_window = nullptr;
    ImGuiContext* m_imgui = nullptr;
    std::unique_ptr<Shader> m_scene;
    std::unique_ptr<Shader> m_blit;
    unsigned m_vao = 0;
    unsigned m_fbo = 0, m_fboTex = 0;
    int m_fboW = 0, m_fboH = 0;
    float m_renderScale = 0.6f; // of the image area, adapted to the GPU
    unsigned m_logoTex = 0;
    ImFont* m_fontTitle = nullptr;
    ImFont* m_fontBody = nullptr;
    ImFont* m_fontSmall = nullptr;
    ImFont* m_fontCaption = nullptr;
    ImFont* m_fontMono = nullptr;

    double m_start = 0.0, m_last = 0.0;
    float m_minSeconds;
    float m_target = 0.0f, m_shown = 0.0f;
    float m_closing = -1.0f; // seconds into the fade-out, < 0 while loading
    struct Step {
        float at;
        std::string status;
    };
    std::vector<Step> m_steps;
};

} // namespace sicdm

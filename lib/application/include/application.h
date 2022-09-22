#pragma once
#include <imgui.h>
#include <string>
#include <array>
#include <memory>

struct Platform;
struct Renderer;

struct Application
{
    Application(const char* name, const char* iniFilename);
    Application(const char* name, const char* iniFilename, int argc, const char* argv[]);
    virtual ~Application();

    bool Create(int width = -1, int height = -1);

    int Run();

    void SetTitle(const char* title);

    bool Close();
    void Quit();

    const std::string& GetName() const;

    ImFont* DefaultFont() const;
    ImFont* WindowFont() const;
    ImFont* MonoFont() const;
    ImFont* HeaderFont() const;

    static void swapDefaultFont(bool big = false);
    static void swapWindowFont(bool big = false);
    static void swapMonoFont(bool big = false);
    static void swapHeaderFont(bool big = false);

    static bool isUsingBigDefaultFont();
    static bool isUsingBigWindowFont();
    static bool isUsingBigMonoFont();
    static bool isUsingBigHeaderFont();

    ImTextureID LoadTexture(const void* data, int len);
    ImTextureID LoadTexture(const char* path);
    ImTextureID CreateTexture(const void* data, int width, int height);
    void DestroyTexture(ImTextureID texture);
    int GetTextureWidth(ImTextureID texture);
    int GetTextureHeight(ImTextureID texture);

    virtual void OnStart() {}
    virtual void OnStop() {}
    virtual bool OnQuitRequest() { return true; }
    virtual void OnFrame(float deltaTime) {}

    virtual ImGuiWindowFlags GetWindowFlags() const;

    virtual bool CanClose() { return true; }

  protected:
    static constexpr std::array<float, 2> defaultFontSize = { 18.F, 38.F };
    static constexpr std::array<float, 2> windowFontSize = { 18.F, 38.F };
    static constexpr std::array<float, 2> monoFontSize = { 18.F, 38.F };
    static constexpr std::array<float, 2> headerFontSize = { 20.F, 42.F };

  private:
    void RecreateFontAtlas();

    void Frame();

    std::string m_Name;
    std::string m_IniFilename;
    std::unique_ptr<Platform> m_Platform;
    std::unique_ptr<Renderer> m_Renderer;
    ImGuiContext* m_Context = nullptr;
    static inline ImFont* m_DefaultFont = nullptr;
    static inline ImFont* m_DefaultFontSmall = nullptr;
    static inline ImFont* m_DefaultFontBig = nullptr;
    static inline ImFont* m_WindowFont = nullptr;
    static inline ImFont* m_WindowFontSmall = nullptr;
    static inline ImFont* m_WindowFontBig = nullptr;
    static inline ImFont* m_MonoFont = nullptr;
    static inline ImFont* m_MonoFontSmall = nullptr;
    static inline ImFont* m_MonoFontBig = nullptr;
    static inline ImFont* m_HeaderFont = nullptr;
    static inline ImFont* m_HeaderFontSmall = nullptr;
    static inline ImFont* m_HeaderFontBig = nullptr;
};

int Main(int argc, const char* argv[]);
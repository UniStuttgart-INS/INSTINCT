#include "application.h"
#include "setup.h"
#include "platform.h"
#include "renderer.h"

#include "implot.h"

extern "C"
{
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#include "stb_image.h"
}

#include "../fonts/PlayRegular.cpp"
#include "../fonts/InconsolataNerdFontComplete.cpp"
#include "../fonts/NotoSansRegular.cpp"
#include "../fonts/CuprumBold.cpp"
#include "../fonts/FreeMono.cpp"

Application::Application(const char* name, const char* iniFilename)
    : Application(name, iniFilename, 0, nullptr)
{
}

Application::Application(const char* name, const char* iniFilename, int argc, const char* argv[])
    : m_Name(name),
      m_IniFilename(iniFilename),
      m_Platform(CreatePlatform(*this)),
      m_Renderer(CreateRenderer())
{
    m_Platform->ApplicationStart(argc, argv);
}

Application::~Application()
{
    m_Renderer->Destroy();

    m_Platform->ApplicationStop();

    if (m_Context)
    {
        ImPlot::DestroyContext();
        ImGui::DestroyContext(m_Context);
        m_Context = nullptr;
    }
}

bool Application::Create(int width /*= -1*/, int height /*= -1*/)
{
    m_Context = ImGui::CreateContext();
    ImGui::SetCurrentContext(m_Context);
    ImPlot::CreateContext();

    if (!m_Platform->OpenMainWindow(m_Name.c_str(), width, height))
        return false;

    if (!m_Renderer->Create(*m_Platform))
        return false;

    auto hasEnding = [](std::string const& fullString, std::string const& ending) -> bool {
        if (fullString.length() >= ending.length())
            return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
        else
            return false;
    };

    if (!hasEnding(m_IniFilename, ".ini"))
        m_IniFilename += ".ini";

    ImGuiIO& io = ImGui::GetIO();
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
    io.IniFilename = m_IniFilename.c_str();
    io.LogFilename = nullptr;

    ImGui::StyleColorsDark();

    RecreateFontAtlas();

    m_Platform->AcknowledgeWindowScaleChanged();
    m_Platform->AcknowledgeFramebufferScaleChanged();

    OnStart();

    Frame();

    return true;
}

int Application::Run()
{
    m_Platform->ShowMainWindow();

    while (m_Platform->ProcessMainWindowEvents())
    {
        if (!m_Platform->IsMainWindowVisible())
            continue;

        Frame();
    }

    OnStop();

    return 0;
}

void Application::RecreateFontAtlas()
{
    ImGuiIO& io = ImGui::GetIO();

    IM_DELETE(io.Fonts);

    io.Fonts = IM_NEW(ImFontAtlas);

    ImFontConfig config;
    config.OversampleH = 4;
    config.OversampleV = 4;
    config.PixelSnapH = false;
    strcpy(config.Name, "Play Regular");

    m_DefaultFont = io.Fonts->AddFontFromMemoryCompressedTTF(PlayRegular_compressed_data, PlayRegular_compressed_size, 18.0F, &config);
    // m_DefaultFont = io.Fonts->AddFontFromFileTTF("resources/fonts/Play-Regular.ttf", 18.0f, &config);
    config.MergeMode = true;
    static const ImWchar icons_ranges[] = { 0x20, 0xFFFF, 0 };
    io.Fonts->AddFontFromMemoryCompressedTTF(InconsolataNerdFontComplete_compressed_data, InconsolataNerdFontComplete_compressed_size, 18.0F, &config, icons_ranges);
    io.Fonts->AddFontFromMemoryCompressedTTF(NotoSansRegular_compressed_data, NotoSansRegular_compressed_size, 18.0F, &config, icons_ranges);
    io.Fonts->AddFontFromMemoryCompressedTTF(FreeMono_compressed_data, FreeMono_compressed_size, 18.0F, &config, icons_ranges);
    // io.Fonts->AddFontFromFileTTF("resources/fonts/Inconsolata-Nerd-Font-Complete.otf", 18.0f, &config, icons_ranges);
    // io.Fonts->AddFontFromFileTTF("resources/fonts/NotoSans-Regular.ttf", 18.0f, &config, icons_ranges);

    config.MergeMode = false;
    strcpy(config.Name, "Inconsolata");
    m_HeaderFont = io.Fonts->AddFontFromMemoryCompressedTTF(InconsolataNerdFontComplete_compressed_data, InconsolataNerdFontComplete_compressed_size, 18.0F, &config);
    strcpy(config.Name, "Cuprum Bold");
    m_HeaderFont = io.Fonts->AddFontFromMemoryCompressedTTF(CuprumBold_compressed_data, CuprumBold_compressed_size, 20.0F, &config);
    // m_HeaderFont = io.Fonts->AddFontFromFileTTF("resources/fonts/Cuprum-Bold.ttf", 20.0f, &config);

    io.Fonts->Build();
}

void Application::Frame()
{
    auto& io = ImGui::GetIO();

    if (m_Platform->HasWindowScaleChanged())
        m_Platform->AcknowledgeWindowScaleChanged();

    if (m_Platform->HasFramebufferScaleChanged())
    {
        RecreateFontAtlas();
        m_Platform->AcknowledgeFramebufferScaleChanged();
    }

    const float windowScale = m_Platform->GetWindowScale();
    const float framebufferScale = m_Platform->GetFramebufferScale();

    if (io.WantSetMousePos)
    {
        io.MousePos.x *= windowScale;
        io.MousePos.y *= windowScale;
    }

    m_Platform->NewFrame();

    // Don't touch "uninitialized" mouse position
    if (io.MousePos.x > -FLT_MAX && io.MousePos.y > -FLT_MAX)
    {
        io.MousePos.x /= windowScale;
        io.MousePos.y /= windowScale;
    }
    io.DisplaySize.x /= windowScale;
    io.DisplaySize.y /= windowScale;

    io.DisplayFramebufferScale.x = framebufferScale;
    io.DisplayFramebufferScale.y = framebufferScale;

    m_Renderer->NewFrame();

    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(io.DisplaySize);
    const auto windowBorderSize = ImGui::GetStyle().WindowBorderSize;
    const auto windowRounding = ImGui::GetStyle().WindowRounding;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::Begin("Content", nullptr, GetWindowFlags());
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, windowBorderSize);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, windowRounding);

    OnFrame(io.DeltaTime);

    ImGui::PopStyleVar(2);
    ImGui::End();
    ImGui::PopStyleVar(2);

    // Rendering
    m_Renderer->Clear(ImColor(32, 32, 32, 255));
    ImGui::Render();
    m_Renderer->RenderDrawData(ImGui::GetDrawData());

    m_Platform->FinishFrame();
}

void Application::SetTitle(const char* title)
{
    m_Platform->SetMainWindowTitle(title);
}

bool Application::Close()
{
    return m_Platform->CloseMainWindow();
}

void Application::Quit()
{
    m_Platform->Quit();
}

const std::string& Application::GetName() const
{
    return m_Name;
}

ImFont* Application::DefaultFont() const
{
    return m_DefaultFont;
}

ImFont* Application::HeaderFont() const
{
    return m_HeaderFont;
}
ImTextureID Application::LoadTexture(const void* data, int len)
{
    int width = 0, height = 0, component = 0;
    if (auto stbi_data = stbi_load_from_memory(reinterpret_cast<const stbi_uc*>(data), len, &width, &height, &component, 4))
    {
        auto texture = CreateTexture(stbi_data, width, height);
        stbi_image_free(stbi_data);
        return texture;
    }
    else
        return nullptr;
}

ImTextureID Application::LoadTexture(const char* path)
{
    int width = 0, height = 0, component = 0;
    if (auto data = stbi_load(path, &width, &height, &component, 4))
    {
        auto texture = CreateTexture(data, width, height);
        stbi_image_free(data);
        return texture;
    }
    else
        return nullptr;
}

ImTextureID Application::CreateTexture(const void* data, int width, int height)
{
    return m_Renderer->CreateTexture(data, width, height);
}

void Application::DestroyTexture(ImTextureID texture)
{
    m_Renderer->DestroyTexture(texture);
}

int Application::GetTextureWidth(ImTextureID texture)
{
    return m_Renderer->GetTextureWidth(texture);
}

int Application::GetTextureHeight(ImTextureID texture)
{
    return m_Renderer->GetTextureHeight(texture);
}

ImGuiWindowFlags Application::GetWindowFlags() const
{
    return ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
}

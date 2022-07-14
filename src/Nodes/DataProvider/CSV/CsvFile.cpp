#include "CsvFile.hpp"

#include "util/Logger.hpp"
#include "util/StringUtil.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"

NAV::CsvFile::CsvFile()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 530, 271 };

    nm::CreateOutputPin(this, CsvData::type().c_str(), Pin::Type::Object, { CsvData::type() }, &_data);
}

NAV::CsvFile::~CsvFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::CsvFile::typeStatic()
{
    return "CsvFile";
}

std::string NAV::CsvFile::type() const
{
    return typeStatic();
}

std::string NAV::CsvFile::category()
{
    return "Data Provider";
}

void NAV::CsvFile::guiConfig()
{
    if (auto res = FileReader::guiConfig(".csv,.*", { ".csv" }, size_t(id), nameId()))
    {
        LOG_DEBUG("{}: Path changed to {}", nameId(), _path);
        flow::ApplyChanges();
        if (res == FileReader::PATH_CHANGED)
        {
            doInitialize();
        }
        else
        {
            doDeinitialize();
        }
    }

    struct TextFilters
    {
        // Cuts off characters if the length exceeds 1
        static int FilterSingleCharacter(ImGuiInputTextCallbackData* data)
        {
            while (data->BufTextLen > 1)
            {
                data->BufDirty = true;
                data->DeleteChars(1, 1);
            }
            return 0;
        }
    };

    std::string tmpStr(1, _delimiter);
    if (ImGui::InputText(fmt::format("Delimiter character##{}", size_t(id)).c_str(), &tmpStr, ImGuiInputTextFlags_CallbackEdit, TextFilters::FilterSingleCharacter))
    {
        _delimiter = tmpStr.empty() ? '\0' : tmpStr.at(0);
        LOG_DEBUG("{}: Delimiter character changed to {}", nameId(), _delimiter);
        flow::ApplyChanges();
        if (_delimiter)
        {
            doInitialize();
        }
    }

    tmpStr = std::string(1, _comment);
    if (ImGui::InputText(fmt::format("Comment character##{}", size_t(id)).c_str(), &tmpStr, ImGuiInputTextFlags_CallbackEdit, TextFilters::FilterSingleCharacter))
    {
        _comment = tmpStr.empty() ? '\0' : tmpStr.at(0);
        LOG_DEBUG("{}: Comment character changed to {}", nameId(), _comment);
        flow::ApplyChanges();
        doInitialize();
    }

    if (ImGui::InputIntL(fmt::format("Skip lines##{}", size_t(id)).c_str(), &_skipLines, 0, std::numeric_limits<int>::max()))
    {
        LOG_DEBUG("{}: Skip lines changed to {}", nameId(), _skipLines);
        flow::ApplyChanges();
        doInitialize();
    }

    if (ImGui::Checkbox(fmt::format("Header line##{}", size_t(id)).c_str(), &_hasHeaderLine))
    {
        LOG_DEBUG("{}: HasHeaderLine changed to {}", nameId(), _hasHeaderLine);
        flow::ApplyChanges();
        doInitialize();
    }

    ImGui::Separator();

    ImGui::Text("Amount of data lines in file: %zu", _data.lines.size());

    // Header info
    if (ImGui::BeginTable(fmt::format("##CSVHeaders ({})", size_t(id)).c_str(), 2,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    {
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Description", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();

        for (size_t i = 0; i < _data.description.size(); i++)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%zu", i);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(_data.description[i].c_str());
        }

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::CsvFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();
    j["delimiter"] = _delimiter;
    j["comment"] = _comment;
    j["skipLines"] = _skipLines;
    j["hasHeaderLine"] = _hasHeaderLine;

    return j;
}

void NAV::CsvFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
    if (j.contains("delimiter"))
    {
        j.at("delimiter").get_to(_delimiter);
    }
    if (j.contains("comment"))
    {
        j.at("comment").get_to(_comment);
    }
    if (j.contains("skipLines"))
    {
        j.at("skipLines").get_to(_skipLines);
    }
    if (j.contains("hasHeaderLine"))
    {
        j.at("hasHeaderLine").get_to(_hasHeaderLine);
    }
}

bool NAV::CsvFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _data.description.clear();
    _data.lines.clear();

    if (!FileReader::initialize())
    {
        return false;
    }

    std::string line;
    while (!_filestream.eof())
    {
        std::getline(_filestream, line);
        if (line.empty() || line.at(0) == _comment) { continue; } // Skip empty and comment lines

        auto splittedData = str::split(line, _delimiter);
        if (!splittedData.empty()) { _data.lines.emplace_back(); }

        for (const auto& cell : splittedData)
        {
            CsvData::CsvElement value;
            try
            {
                value = std::stod(cell);
            }
            catch (...)
            {
                value = cell;
            }
            _data.lines.back().push_back(value);
        }
    }

    return true;
}

void NAV::CsvFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    _data.description.clear();
    _data.lines.clear();

    FileReader::deinitialize();
}

NAV::FileReader::FileType NAV::CsvFile::determineFileType()
{
    return FileReader::FileType::CSV;
}

void NAV::CsvFile::readHeader()
{
    for (int i = 0; i < _skipLines; i++) { _filestream.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); } // Skip lines at the start of the file

    std::string line;
    if (_hasHeaderLine)
    {
        std::getline(_filestream, line);
        _data.description = str::split(line, _delimiter);
        for (auto& desc : _data.description)
        {
            desc.erase(std::find_if(desc.begin(), desc.end(), [](int ch) { return std::iscntrl(ch); }), desc.end());
        }
    }
}
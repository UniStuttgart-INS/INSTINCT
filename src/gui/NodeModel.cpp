#include "NodeModel.hpp"

#include <QtWidgets/QLabel>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QComboBox>
#include <QAbstractItemView>

#include <tuple>
#include <iostream>
#include <sstream>
#include <regex>

#include "util/Logger.hpp"

void NodeModel::updateView(QSpinBox* inputSpinBox)
{
    LOG_TRACE("called");

    if (getParentNode() && getParentNode()->getNodeGraphicsObject())
    {
        // Add/Remove Connection Ports
        if (getParentNode()->nodeState().getEntries(PortType::In).size() > nPorts(PortType::In)
            && !getParentNode()->nodeState().getEntries(PortType::In).at(getParentNode()->nodeState().getEntries(PortType::In).size() - 1).empty())
        {
            inputSpinBox->setValue(static_cast<int>(getParentNode()->nodeState().getEntries(PortType::In).size()));
            return;
        }
        for (size_t i = 0; getParentNode()->nodeState().getEntries(PortType::In).size() - i > nPorts(PortType::In); i++)
        {
            removeRepeatedConfigGroupBox(getParentNode()->nodeState().getEntries(PortType::In).size() - i);
        }
        for (size_t j = 0; getParentNode()->nodeState().getEntries(PortType::In).size() + j < nPorts(PortType::In); j++)
        {
            const auto& nodeInfo = nodeManager.registeredNodeTypes().find(_name.toStdString())->second;
            const auto& guiConfigs = nodeInfo.constructorEmpty()->guiConfig();
            for (size_t i = 0; i < guiConfigs.size(); i++)
            {
                const auto& config = guiConfigs.at(i);

                if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_N_INPUT_PORTS)
                {
                    size_t configRepeatedNumber = std::stoul(std::get<std::string>(std::get<3>(config).at(3)));
                    addRepeatedConfigGroupBox(guiConfigs, _mainLayout, getParentNode()->nodeState().getEntries(PortType::In).size() + 1 + j, i, configRepeatedNumber);
                }
            }
        }
        getParentNode()->nodeState().getEntries(PortType::In).resize(nPorts(PortType::In));

        getParentNode()->nodeGraphicsObject().setGeometryChanged();
        getParentNode()->nodeGeometry().recalculateSize();
        getParentNode()->nodeGraphicsObject().update();
        getParentNode()->nodeGraphicsObject().moveConnections();
    }
}

void NodeModel::updateVariants(const std::vector<NAV::Node::ConfigOptions>& guiConfigs, const size_t configPosition, QFormLayout* layout, int layoutPosition)
{
    LOG_TRACE("updateVariants configPosition={}, layoutPosition={}", configPosition, layoutPosition);

    size_t itemsToUpdate = 0;
    for (size_t i = configPosition + 1; i < guiConfigs.size(); i++)
    {
        if (std::get<0>(guiConfigs.at(i)) == NAV::Node::ConfigOptionType::CONFIG_VARIANT)
        {
            itemsToUpdate++;
        }
        else
        {
            break;
        }
    }

    if (layout->rowCount() - layoutPosition - 1 < static_cast<int>(itemsToUpdate))
    {
        LOG_CRITICAL("The Layout somehow has less items than variants");
    }

    LOG_DEBUG("{} Variants to Update", itemsToUpdate);
    for (size_t i = 1; i <= itemsToUpdate; i++)
    {
        auto& configToUpdate = guiConfigs.at(configPosition + i);
        int layoutRowToUpdate = layoutPosition + static_cast<int>(i);
        QString objName = layout->itemAt(layoutRowToUpdate, QFormLayout::ItemRole::FieldRole)->widget()->objectName();
        QString prefix;
        if (objName.indexOf('-') <= 2)
        {
            prefix = objName.left(objName.indexOf('-') + 1);
        }

        LOG_DEBUG("layoutRowToUpdate={} with prefix={}", layoutRowToUpdate, prefix.toStdString());
        layout->removeRow(layoutRowToUpdate);
        addGuiElementForConfig(configToUpdate, guiConfigs, configPosition + i, layout, layoutRowToUpdate, prefix);
    }
}

void NodeModel::addListRow(std::vector<std::string> configOptions, QFormLayout* layout, QGroupBox* gridGroupBox)
{
    LOG_TRACE("called");

    QSpinBox* spinBox = static_cast<QSpinBox*>(layout->itemAt(0, QFormLayout::ItemRole::FieldRole)->widget());
    int currentRows = layout->rowCount() - 1;

    LOG_DEBUG("CurrentRows={}, spinBox={}", currentRows, spinBox->value());
    while (spinBox->value() > currentRows)
    {
        QComboBox* comboBox = new QComboBox(gridGroupBox);
        comboBox->setStyleSheet("background-color: rgb(220,220,220); color: black; selection-background-color: rgb(169,169,169); combobox-popup: 0");

        for (auto& cell : configOptions)
        {
            if (cell.at(0) == '[')
            {
                comboBox->addItem(QString::fromStdString(cell.substr(1, cell.size() - 2)));
                comboBox->setCurrentIndex(comboBox->count() - 1);
            }
            else
                comboBox->addItem(QString::fromStdString(cell));
        }

        LOG_DEBUG("Inserting widget at row={}", currentRows + 1);
        layout->addRow(QString::fromStdString("[" + std::to_string(currentRows + 1) + "]"), comboBox);

        currentRows = layout->rowCount() - 1;
    }
    while (spinBox->value() < currentRows)
    {
        LOG_DEBUG("Removing widgets at row={}", currentRows);
        layout->removeRow(currentRows);

        currentRows = layout->rowCount() - 1;
    }
}

void NodeModel::addListListRow(std::vector<std::string> configOptions, QGridLayout* layout, QGroupBox* gridGroupBox)
{
    LOG_TRACE("called");

    QSpinBox* spinBox = static_cast<QSpinBox*>(layout->itemAtPosition(0, 1)->widget());
    int currentRows = layout->count() / 2 - 2;

    LOG_DEBUG("CurrentRows={}, spinBox={}", currentRows, spinBox->value());
    while (spinBox->value() > currentRows)
    {
        size_t c = 0;
        std::array<QComboBox*, 2> comboBoxes;
        for (size_t i = 0; i < comboBoxes.size(); i++)
        {
            comboBoxes.at(i) = new QComboBox(gridGroupBox);
            comboBoxes.at(i)->setStyleSheet("background-color: rgb(220,220,220); color: black; selection-background-color: rgb(169,169,169); combobox-popup: 0");

            while (configOptions.size() > c && configOptions.at(c) != "|")
            {
                if (configOptions.at(c).at(0) == '[')
                {
                    comboBoxes.at(i)->addItem(QString::fromStdString(configOptions.at(c).substr(1, configOptions.at(c).size() - 2)));
                    comboBoxes.at(i)->setCurrentIndex(comboBoxes.at(i)->count() - 1);
                }
                else
                {
                    comboBoxes.at(i)->addItem(QString::fromStdString(configOptions.at(c)));
                }

                c++;
            }
            LOG_DEBUG("Inserting widget at row={}, col={}", currentRows + 2, i);
            layout->addWidget(comboBoxes.at(i), currentRows + 2, static_cast<int>(i));
            c++;
        }
        currentRows = layout->count() / 2 - 2;
    }
    while (spinBox->value() < currentRows)
    {
        LOG_DEBUG("Removing widgets at row={}", currentRows + 1);
        delete layout->itemAtPosition(currentRows + 1, 1)->widget();
        delete layout->itemAtPosition(currentRows + 1, 0)->widget();
        layout->removeItem(layout->itemAtPosition(currentRows + 1, 1));
        layout->removeItem(layout->itemAtPosition(currentRows + 1, 0));
        currentRows = layout->count() / 2 - 2;
    }
}

void NodeModel::addGuiElementForConfig(const NAV::Node::ConfigOptions& config, const std::vector<NAV::Node::ConfigOptions>& guiConfigs, const size_t configPosition, QFormLayout* _layout, int layoutInsertPosition, QString prefix)
{
    LOG_TRACE("called");

    QString description = QString::fromStdString(std::regex_replace(std::get<1>(config), std::regex("^\\d-"), ""));
    QString label = QString::fromStdString(std::get<1>(config));

    if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_BOOL)
    {
        LOG_TRACE("Type == BOOL");
        QCheckBox* checkBox = new QCheckBox();

        checkBox->setChecked(std::stoi(std::get<std::string>(std::get<3>(config).front())));
        checkBox->setStyleSheet("QCheckBox::indicator:unchecked { border: 1px solid rgb(220,220,220); }");
        _layout->insertRow(layoutInsertPosition, description, checkBox);

        connect(checkBox, QOverload<int>::of(&QCheckBox::stateChanged),
                [this, guiConfigs, configPosition, _layout, layoutInsertPosition]() {
                    this->updateVariants(guiConfigs, configPosition, _layout, layoutInsertPosition);
                });
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_INT
             || std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_N_INPUT_PORTS)
    {
        LOG_TRACE("Type == INT || N_INPUT_PORTS");
        QSpinBox* spinBox = new QSpinBox();

        int defaultValue = 0;
        for (size_t j = 0; j < 3; j++)
        {
            std::string cell = std::get<std::string>(std::get<3>(config).at(j));
            if (j == 0)
                spinBox->setMinimum(std::stoi(cell));
            else if (j == 1)
                defaultValue = std::stoi(cell);
            else if (j == 2)
            {
                spinBox->setMaximum(std::stoi(cell));
                spinBox->setValue(defaultValue);
            }
        }

        if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_N_INPUT_PORTS)
        {
            connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged),
                    [this, spinBox]() {
                        this->updateView(spinBox);
                    });
        }

        spinBox->setSingleStep(1);
        spinBox->setStyleSheet("QSpinBox { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
        _layout->insertRow(layoutInsertPosition, description, spinBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_FLOAT)
    {
        LOG_TRACE("Type == FLOAT");
        QDoubleSpinBox* doubleSpinBox = new QDoubleSpinBox();

        double defaultValue = 0.0;
        for (size_t j = 0; j < 3; j++)
        {
            std::string cell = std::get<std::string>(std::get<3>(config).at(j));
            if (j == 0)
                doubleSpinBox->setMinimum(std::stod(cell));
            else if (j == 1)
                defaultValue = std::stod(cell);
            else if (j == 2)
            {
                doubleSpinBox->setMaximum(std::stod(cell));
                doubleSpinBox->setValue(defaultValue);
            }
        }

        doubleSpinBox->setSingleStep(1.0);
        doubleSpinBox->setStyleSheet("QDoubleSpinBox { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
        _layout->insertRow(layoutInsertPosition, description, doubleSpinBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_FLOAT3)
    {
        LOG_TRACE("Type == FLOAT3");

        QGroupBox* groupBox = new QGroupBox();
        QHBoxLayout* layout = new QHBoxLayout();

        QDoubleSpinBox* doubleSpinBox[3];

        for (size_t sb = 0; sb < 3; sb++)
        {
            doubleSpinBox[sb] = new QDoubleSpinBox();

            double defaultValue = 0.0;
            for (size_t j = 0; j < 3; j++)
            {
                std::string cell = std::get<std::string>(std::get<3>(config).at(j + 3 * sb));
                if (j == 0)
                    doubleSpinBox[sb]->setMinimum(std::stod(cell));
                else if (j == 1)
                    defaultValue = std::stod(cell);
                else if (j == 2)
                {
                    doubleSpinBox[sb]->setMaximum(std::stod(cell));
                    doubleSpinBox[sb]->setValue(defaultValue);
                }
            }
            doubleSpinBox[sb]->setSingleStep(1.0);
            doubleSpinBox[sb]->setDecimals(10);
            doubleSpinBox[sb]->setStyleSheet("QDoubleSpinBox { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");

            layout->addWidget(doubleSpinBox[sb]);
        }

        layout->setContentsMargins(0, 0, 0, 0);
        groupBox->setLayout(layout);
        groupBox->setStyleSheet("border:0;");
        // groupBox->setFlat(true);
        _layout->insertRow(layoutInsertPosition, description, groupBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_STRING)
    {
        LOG_TRACE("Type == STRING");
        QLineEdit* lineEdit = new QLineEdit();

        lineEdit->setStyleSheet("QLineEdit { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
        lineEdit->setText(QString::fromStdString(std::get<std::string>(std::get<3>(config).front())));
        _layout->insertRow(layoutInsertPosition, description, lineEdit);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_STRING_BOX)
    {
        LOG_TRACE("Type == STRING_BOX");
        QGroupBox* groupBox = new QGroupBox(description);
        QVBoxLayout* layout = new QVBoxLayout();

        QTextEdit* textEdit = new QTextEdit(groupBox);
        textEdit->setStyleSheet("QTextEdit { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");

        connect(textEdit, &QTextEdit::textChanged,
                [textEdit]() {
                    LOG_TRACE("called");

                    auto font = textEdit->document()->defaultFont();
                    auto fontMetrics = QFontMetrics(font);
                    auto textSize = fontMetrics.size(0, textEdit->toPlainText());

                    auto textWidth = textSize.width() + 20;
                    auto textHeight = textSize.height() + 20;

                    LOG_DEBUG("Setting QTextEdit size to w: {}; h: {}", textWidth, textHeight);
                    textEdit->setMinimumSize(textWidth, textHeight);
                });

        textEdit->setText(QString::fromStdString(std::get<std::string>(std::get<3>(config).front())));

        layout->addWidget(textEdit);
        groupBox->setLayout(layout);

        _layout->insertRow(layoutInsertPosition, groupBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_LIST)
    {
        LOG_TRACE("Type == LIST");
        QComboBox* comboBox = new QComboBox();

        for (auto& cellVariant : std::get<3>(config))
        {
            auto& cell = std::get<std::string>(cellVariant);
            if (cell.at(0) == '[')
            {
                comboBox->addItem(QString::fromStdString(cell.substr(1, cell.size() - 2)));
                comboBox->setCurrentIndex(comboBox->count() - 1);
            }
            else
                comboBox->addItem(QString::fromStdString(cell));
        }

        comboBox->setStyleSheet("background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black; combobox-popup: 0");
        _layout->insertRow(layoutInsertPosition, description, comboBox);

        connect(comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
                [this, guiConfigs, configPosition, _layout, layoutInsertPosition]() {
                    this->updateVariants(guiConfigs, configPosition, _layout, layoutInsertPosition);
                });
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_LIST_MULTI)
    {
        LOG_TRACE("Type == LIST_MULTI");
        QGroupBox* gridGroupBox = new QGroupBox(description);
        QFormLayout* layout = new QFormLayout();

        QSpinBox* spinBox = new QSpinBox(gridGroupBox);
        spinBox->setRange(0, 20);
        spinBox->setValue(0);
        spinBox->setSingleStep(1);
        spinBox->setStyleSheet("QSpinBox { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
        spinBox->setObjectName(prefix + description + "-Count");
        layout->addRow("Data lines", spinBox);

        std::vector<std::string> stringOptions;
        for (const auto& conf : std::get<3>(config))
        {
            stringOptions.push_back(std::get<std::string>(conf));
        }

        connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged),
                [this, stringOptions, layout, gridGroupBox]() {
                    this->addListRow(stringOptions, layout, gridGroupBox);
                });

        gridGroupBox->setLayout(layout);
        _layout->insertRow(layoutInsertPosition, gridGroupBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_MULTI)
    {
        LOG_TRACE("Type == LIST_LIST_MULTI");
        QGroupBox* gridGroupBox = new QGroupBox(description);
        QGridLayout* layout = new QGridLayout();

        layout->addWidget(new QLabel("Data lines", gridGroupBox), 0, 0);
        QSpinBox* spinBox = new QSpinBox(gridGroupBox);
        spinBox->setRange(0, 20);
        spinBox->setValue(0);
        spinBox->setSingleStep(1);
        spinBox->setStyleSheet("QSpinBox { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
        spinBox->setObjectName(prefix + description + "-Count");
        layout->addWidget(spinBox, 0, 1);

        layout->addWidget(new QLabel("X Data Source", gridGroupBox), 1, 0);
        layout->addWidget(new QLabel("Y Data Source", gridGroupBox), 1, 1);

        std::vector<std::string> stringOptions;
        for (const auto& conf : std::get<3>(config))
        {
            stringOptions.push_back(std::get<std::string>(conf));
        }

        connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged),
                [this, stringOptions, layout, gridGroupBox]() {
                    this->addListListRow(stringOptions, layout, gridGroupBox);
                });

        gridGroupBox->setLayout(layout);
        _layout->insertRow(layoutInsertPosition, gridGroupBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_MAP_INT)
    {
        LOG_TRACE("Type == MAP_INT");
        QSpinBox* spinBox = new QSpinBox();

        int defaultValue = 0;
        for (size_t j = 0; j < 4; j++)
        {
            std::string cell = std::get<std::string>(std::get<3>(config).at(j));
            if (j == 0)
                spinBox->setProperty("key", QString::fromStdString(cell));
            if (j == 1)
                spinBox->setMinimum(std::stoi(cell));
            else if (j == 2)
                defaultValue = std::stoi(cell);
            else if (j == 3)
            {
                spinBox->setMaximum(std::stoi(cell));
                spinBox->setValue(defaultValue);
            }
        }

        spinBox->setSingleStep(1);
        spinBox->setStyleSheet("QSpinBox { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
        _layout->insertRow(layoutInsertPosition, description, spinBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_VARIANT)
    {
        LOG_TRACE("Type == CONFIG_VARIANT");
        if (configPosition > 0)
        {
            int selection = -1;
            const auto& prevConfig = guiConfigs.at(configPosition - 1);

            if (std::get<0>(prevConfig) == NAV::Node::ConfigOptionType::CONFIG_LIST)
            {
                LOG_DEBUG("Variant Prev Item List={}", _layout->itemAt(layoutInsertPosition - 1, QFormLayout::ItemRole::FieldRole)->widget()->objectName().toStdString());
                QComboBox* comboBox = static_cast<QComboBox*>(_layout->itemAt(layoutInsertPosition - 1, QFormLayout::ItemRole::FieldRole)->widget());
                selection = comboBox->currentIndex();
            }
            else if (std::get<0>(prevConfig) == NAV::Node::ConfigOptionType::CONFIG_BOOL)
            {
                LOG_DEBUG("Variant Prev Item Bool={}", _layout->itemAt(layoutInsertPosition - 1, QFormLayout::ItemRole::FieldRole)->widget()->objectName().toStdString());
                QCheckBox* checkBox = static_cast<QCheckBox*>(_layout->itemAt(layoutInsertPosition - 1, QFormLayout::ItemRole::FieldRole)->widget());
                selection = static_cast<int>(checkBox->isChecked());
            }
            else if (std::get<0>(prevConfig) == NAV::Node::ConfigOptionType::CONFIG_VARIANT)
            {
                LOG_DEBUG("Variant Prev Item Variant={}", _layout->itemAt(layoutInsertPosition - 1, QFormLayout::ItemRole::FieldRole)->widget()->objectName().toStdString());
                selection = _layout->itemAt(layoutInsertPosition - 1, QFormLayout::ItemRole::FieldRole)->widget()->property("variant-selection").toInt();
            }
            else
            {
                LOG_CRITICAL("Variants preceeding item is of type {}, which is not supported.", std::get<0>(prevConfig));
            }

            // Selection found
            LOG_TRACE("{} has selected {}", _layout->itemAt(layoutInsertPosition - 1, QFormLayout::ItemRole::FieldRole)->widget()->objectName().toStdString(), selection);
            const auto& selectedConfigBase = std::get<NAV::Node::ConfigOptionsBase>(std::get<3>(config).at(static_cast<size_t>(selection)));
            std::vector<std::variant<std::string, NAV::Node::ConfigOptionsBase>> elementOptions(std::get<3>(selectedConfigBase).begin(), std::get<3>(selectedConfigBase).end());

            NAV::Node::ConfigOptions selectedConfig = { std::get<0>(selectedConfigBase), std::get<1>(selectedConfigBase), std::get<2>(selectedConfigBase), elementOptions };
            addGuiElementForConfig(selectedConfig, guiConfigs, configPosition, _layout, layoutInsertPosition, prefix);
            _layout->itemAt(layoutInsertPosition, QFormLayout::ItemRole::FieldRole)->widget()->setProperty("variant-selection", selection);
        }
        else
        {
            LOG_CRITICAL("Variants need a preceeding item which gives a choice");
        }
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_EMPTY)
    {
        LOG_TRACE("Type == EMPTY");
        _layout->insertRow(layoutInsertPosition, "", new QLabel());
    }

    if (std::get<0>(config) != NAV::Node::ConfigOptionType::CONFIG_VARIANT)
    {
        if (auto layoutItem = _layout->itemAt(layoutInsertPosition, QFormLayout::ItemRole::LabelRole))
        {
            if (QWidget* widget = layoutItem->widget())
            {
                widget->setToolTip(QString::fromStdString(std::get<2>(config)));
            }
        }

        if (auto layoutItem = _layout->itemAt(layoutInsertPosition, QFormLayout::ItemRole::FieldRole))
        {
            if (QWidget* widget = layoutItem->widget())
            {
                widget->setObjectName(prefix + label);
                widget->setProperty("type", std::get<0>(config));
                widget->setToolTip(QString::fromStdString(std::get<2>(config)));
                LOG_DEBUG("Added GUI Element of type={} with Name={}", std::get<0>(config), (prefix + description).toStdString());
            }
        }
    }
}

void NodeModel::addRepeatedConfigGroupBox(const std::vector<NAV::Node::ConfigOptions>& guiConfigs,
                                          QFormLayout* _layout,
                                          size_t portNumber,
                                          size_t configRepeatedStart,
                                          size_t configRepeatedNumber)
{
    LOG_TRACE("called");

    QString description = QString::fromStdString("Port " + std::to_string(portNumber));
    QGroupBox* gridGroupBox = new QGroupBox(description);
    QFormLayout* layout = new QFormLayout();
    gridGroupBox->setObjectName(description);

    for (size_t k = 0; k < configRepeatedNumber; k++)
    {
        const auto& config = guiConfigs.at(configRepeatedStart + 1 + k);
        addGuiElementForConfig(config, guiConfigs, configRepeatedStart + 1 + k, layout, layout->rowCount(), QString::fromStdString(std::to_string(portNumber) + "-"));
    }

    gridGroupBox->setLayout(layout);
    _layout->addRow(gridGroupBox);
}

void NodeModel::clearLayout(QFormLayout* layout)
{
    LOG_TRACE("called");

    while (layout->rowCount())
    {
        QLayoutItem* item = layout->itemAt(0, QFormLayout::ItemRole::FieldRole);
        if (QWidget* widget = item->widget())
        {
            widget->deleteLater();
        }
        layout->removeRow(0);
    }
}

void NodeModel::removeRepeatedConfigGroupBox(size_t portNumber)
{
    LOG_TRACE("called");

    for (int i = 0; i < _mainLayout->rowCount(); i++)
    {
        if (_mainLayout->itemAt(i, QFormLayout::ItemRole::FieldRole)->widget()->objectName() == QString::fromStdString("Port " + std::to_string(portNumber)))
        {
            QGroupBox* gridGroupBox = static_cast<QGroupBox*>(_mainLayout->itemAt(i, QFormLayout::ItemRole::FieldRole)->widget());

            clearLayout(static_cast<QFormLayout*>(gridGroupBox->layout()));

            _mainLayout->removeRow(i);

            break;
        }
    }
}

NodeModel::NodeModel(QString const& name)
    : _name(name), _mainWidget(new QWidget())
{
    LOG_TRACE("called for {}", name.toStdString());

    _mainWidget->setStyleSheet("QWidget { background-color: transparent; color: white }");
    _mainLayout = new QFormLayout(_mainWidget);
    _mainLayout->setContentsMargins(0, 5, 0, 0);
    _mainLayout->setLabelAlignment(Qt::AlignRight);

    const auto& nodeInfo = nodeManager.registeredNodeTypes().find(name.toStdString())->second;
    const auto& guiConfigs = nodeInfo.constructorEmpty()->guiConfig();

    for (size_t i = 0; i < guiConfigs.size(); i++)
    {
        const auto& config = guiConfigs.at(i);

        addGuiElementForConfig(config, guiConfigs, i, _mainLayout, _mainLayout->rowCount());

        if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_N_INPUT_PORTS)
        {
            size_t configRepeatedNumber = std::stoul(std::get<std::string>(std::get<3>(config).at(3)));

            auto minPort = std::stoul(std::get<std::string>(std::get<3>(config).at(0)));
            auto nPort = std::stoul(std::get<std::string>(std::get<3>(config).at(1)));

            for (size_t j = minPort - 1; j < nPort; j++)
            {
                addRepeatedConfigGroupBox(guiConfigs, _mainLayout, j + 1, i, configRepeatedNumber);
            }

            i += configRepeatedNumber;
        }
    }
}

QString NodeModel::caption() const
{
    return _name;
}

QString NodeModel::name() const
{
    return _name;
}

unsigned int NodeModel::nPorts(PortType portType) const
{
    const auto& nodeInfo = nodeManager.registeredNodeTypes().find(_name.toStdString())->second;

    if (portType == PortType::In)
    {
        for (int i = 0; i < _mainLayout->rowCount(); i++)
        {
            QWidget* widget = _mainLayout->itemAt(i, QFormLayout::ItemRole::FieldRole)->widget();
            if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_N_INPUT_PORTS)
            {
                return static_cast<unsigned int>(static_cast<QSpinBox*>(widget)->value());
            }
        }

        return static_cast<unsigned int>(nodeInfo.constructorEmpty()->nPorts(NAV::Node::PortType::In));
    }
    else if (portType == PortType::Out)
    {
        return static_cast<unsigned int>(nodeInfo.constructorEmpty()->nPorts(NAV::Node::PortType::Out));
    }

    return 0;
}

NodeDataType determinePortTypeForLayout(QFormLayout* layout, PortIndex portIndex, PortType portType)
{
    LOG_TRACE("called for portIndex={}, portType={}", portIndex, portType);
    for (int i = 0; i < layout->rowCount(); i++)
    {
        QWidget* widget = layout->itemAt(i, QFormLayout::ItemRole::FieldRole)->widget();

        if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST)
        {
            if ((portType == PortType::In && widget->objectName() == QString::fromStdString(std::to_string(portIndex + 1) + "-Input Port Type"))
                || (portType == PortType::Out && widget->objectName() == QString::fromStdString(std::to_string(portIndex + 1) + "-Output Port Type"))
                || (widget->objectName() == QString::fromStdString(std::to_string(portIndex + 1) + "-Port Type")))
            {
                return { static_cast<QComboBox*>(widget)->currentText(),
                         static_cast<QComboBox*>(widget)->currentText() };
            }
        }
        if (widget->layout()
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_LIST_MULTI
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_MULTI
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_STRING_BOX
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_FLOAT3)
        {
            auto portTypeFromLayout = determinePortTypeForLayout(static_cast<QFormLayout*>(widget->layout()), portIndex, portType);
            if (portTypeFromLayout.name != "")
            {
                return portTypeFromLayout;
            }
        }
    }
    return { "", "" };
}

NodeDataType NodeModel::dataType(PortType portType, PortIndex portIndex) const
{
    LOG_TRACE("called for portIndex={}", portIndex);
    const auto& nodeInfo = nodeManager.registeredNodeTypes().find(_name.toStdString())->second;

    uint8_t port = static_cast<uint8_t>(portIndex);

    auto portTypeFromLayout = determinePortTypeForLayout(_mainLayout, portIndex, portType);
    if (portTypeFromLayout.name != "")
    {
        return portTypeFromLayout;
    }

    if (portType == PortType::In)
    {
        return { QString::fromStdString(std::string(nodeInfo.constructorEmpty()->dataType(NAV::Node::PortType::In, port))),
                 QString::fromStdString(std::string(nodeInfo.constructorEmpty()->dataType(NAV::Node::PortType::In, port))) };
    }
    else if (portType == PortType::Out)
    {
        return { QString::fromStdString(std::string(nodeInfo.constructorEmpty()->dataType(NAV::Node::PortType::Out, port))),
                 QString::fromStdString(std::string(nodeInfo.constructorEmpty()->dataType(NAV::Node::PortType::Out, port))) };
    }

    return NodeDataType();
}

void NodeModel::saveLayoutItems(QFormLayout* layout, QJsonObject& modelJson) const
{
    LOG_DEBUG("Items in QFormLayout: {}", layout->rowCount());

    for (int i = 0; i < layout->rowCount(); i++)
    {
        QWidget* widget = layout->itemAt(i, QFormLayout::ItemRole::FieldRole)->widget();

        if (widget->layout()
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_LIST_MULTI
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_MULTI
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_STRING_BOX
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_FLOAT3)
        {
            saveLayoutItems(static_cast<QFormLayout*>(widget->layout()), modelJson);
        }
        else
        {
            LOG_DEBUG("Saving item (type={}): {}", widget->property("type").toUInt(), widget->objectName().toStdString());
            if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_BOOL)
                modelJson[widget->objectName()] = static_cast<QCheckBox*>(widget)->isChecked();
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_INT)
                modelJson[widget->objectName()] = static_cast<QSpinBox*>(widget)->value();
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_FLOAT)
                modelJson[widget->objectName()] = static_cast<QDoubleSpinBox*>(widget)->value();
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_FLOAT3)
            {
                auto* groupBox = static_cast<QGroupBox*>(widget);
                auto* layout = static_cast<QHBoxLayout*>(groupBox->layout());

                auto* spinBox0 = static_cast<QDoubleSpinBox*>(layout->itemAt(0)->widget());
                auto* spinBox1 = static_cast<QDoubleSpinBox*>(layout->itemAt(1)->widget());
                auto* spinBox2 = static_cast<QDoubleSpinBox*>(layout->itemAt(2)->widget());

                modelJson[widget->objectName()] = QString::number(spinBox0->value(), 'g', 13)
                                                  + "," + QString::number(spinBox1->value(), 'g', 13)
                                                  + "," + QString::number(spinBox2->value(), 'g', 13);
            }
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_STRING)
                modelJson[widget->objectName()] = static_cast<QLineEdit*>(widget)->text();
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_STRING_BOX)
            {
                auto groupBox = static_cast<QGroupBox*>(widget);
                auto layout = static_cast<QVBoxLayout*>(groupBox->layout());

                QTextEdit* textEdit = static_cast<QTextEdit*>(layout->itemAt(0)->widget());

                modelJson[widget->objectName()] = textEdit->toPlainText();
            }
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST)
                modelJson[widget->objectName()] = static_cast<QComboBox*>(widget)->currentText();
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST_MULTI)
            {
                auto groupBox = static_cast<QGroupBox*>(widget);
                auto layout = static_cast<QFormLayout*>(groupBox->layout());

                QSpinBox* spinBox = static_cast<QSpinBox*>(layout->itemAt(0, QFormLayout::ItemRole::FieldRole)->widget());
                std::string json = std::to_string(spinBox->value());

                for (int j = 1; j < layout->rowCount(); j++)
                {
                    QComboBox* list = static_cast<QComboBox*>(layout->itemAt(j, QFormLayout::ItemRole::FieldRole)->widget());

                    json += ";" + list->currentText().toStdString();
                }
                modelJson[widget->objectName()] = QString::fromStdString(json);
            }
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_MULTI)
            {
                auto gridGroupBox = static_cast<QGroupBox*>(widget);
                auto layout = static_cast<QGridLayout*>(gridGroupBox->layout());

                QSpinBox* spinBox = static_cast<QSpinBox*>(layout->itemAtPosition(0, 1)->widget());
                std::string json = std::to_string(spinBox->value());

                for (int j = 2; j < layout->count() / 2; j++)
                {
                    QComboBox* xlist = static_cast<QComboBox*>(layout->itemAtPosition(j, 0)->widget());
                    QComboBox* ylist = static_cast<QComboBox*>(layout->itemAtPosition(j, 1)->widget());

                    json += ";" + xlist->currentText().toStdString() + "|" + ylist->currentText().toStdString();
                }
                modelJson[widget->objectName()] = QString::fromStdString(json);
            }
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_MAP_INT)
                modelJson[widget->objectName()] = static_cast<QSpinBox*>(widget)->value();
        }
    }
}

QJsonObject NodeModel::save() const
{
    QJsonObject modelJson = NodeDataModel::save();

    saveLayoutItems(_mainLayout, modelJson);

    return modelJson;
}

void NodeModel::restoreLayoutItems(QFormLayout* layout, QJsonObject const& p)
{
    LOG_DEBUG("Items in QFormLayout: {}", layout->rowCount());

    for (int i = 0; i < layout->rowCount(); i++)
    {
        QWidget* widget = layout->itemAt(i, QFormLayout::ItemRole::FieldRole)->widget();
        if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_N_INPUT_PORTS)
        {
            static_cast<QSpinBox*>(widget)->setValue(p["nInputPorts"].toInt());
            getParentNode()->nodeState().getEntries(PortType::In).resize(static_cast<size_t>(p["nInputPorts"].toInt()));
            break;
        }
    }

    for (int i = 0; i < layout->rowCount(); i++)
    {
        QWidget* widget = layout->itemAt(i, QFormLayout::ItemRole::FieldRole)->widget();

        if (widget->layout()
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_LIST_MULTI
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_MULTI
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_STRING_BOX
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_FLOAT3)
        {
            restoreLayoutItems(static_cast<QFormLayout*>(widget->layout()), p);
        }
        else
        {
            LOG_DEBUG("Restoring item (type={}): {}", widget->property("type").toUInt(), widget->objectName().toStdString());

            QJsonValue v = p[widget->objectName()];
            if (!v.isUndefined())
            {
                if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_BOOL)
                    static_cast<QCheckBox*>(widget)->setChecked(v.toBool());
                else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_INT)
                    static_cast<QSpinBox*>(widget)->setValue(v.toInt());
                else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_FLOAT)
                    static_cast<QDoubleSpinBox*>(widget)->setValue(v.toDouble());
                else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_FLOAT3)
                {
                    auto* groupBox = static_cast<QGroupBox*>(widget);
                    auto* layout = static_cast<QHBoxLayout*>(groupBox->layout());

                    auto* spinBox0 = static_cast<QDoubleSpinBox*>(layout->itemAt(0)->widget());
                    auto* spinBox1 = static_cast<QDoubleSpinBox*>(layout->itemAt(1)->widget());
                    auto* spinBox2 = static_cast<QDoubleSpinBox*>(layout->itemAt(2)->widget());

                    QStringList stringList = v.toString().split(",");

                    spinBox0->setValue(stringList.at(0).toDouble());
                    spinBox1->setValue(stringList.at(1).toDouble());
                    spinBox2->setValue(stringList.at(2).toDouble());
                }
                else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_STRING)
                    static_cast<QLineEdit*>(widget)->setText(v.toString());
                else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_STRING_BOX)
                {
                    auto groupBox = static_cast<QGroupBox*>(widget);
                    auto layout = static_cast<QVBoxLayout*>(groupBox->layout());

                    QTextEdit* textEdit = static_cast<QTextEdit*>(layout->itemAt(0)->widget());
                    textEdit->setText(v.toString());
                }
                else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST)
                    static_cast<QComboBox*>(widget)->setCurrentText(v.toString());
                else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST_MULTI)
                {
                    auto groupBox = static_cast<QGroupBox*>(widget);
                    auto layout = static_cast<QFormLayout*>(groupBox->layout());

                    QSpinBox* spinBox = static_cast<QSpinBox*>(layout->itemAt(0, QFormLayout::ItemRole::FieldRole)->widget());

                    std::string json = v.toString().toStdString();
                    std::stringstream lineStream(json);
                    std::string line;
                    for (int j = 0; std::getline(lineStream, line, ';'); j++)
                    {
                        if (j == 0)
                        {
                            LOG_DEBUG("Setting Spinbox Value to {}", std::stoi(line));
                            spinBox->setValue(std::stoi(line));
                            continue;
                        }
                        LOG_DEBUG("Setting List(row={}) value to {}", j, line);
                        static_cast<QComboBox*>(layout->itemAt(j, QFormLayout::ItemRole::FieldRole)->widget())->setCurrentText(QString::fromStdString(line));
                    }
                }
                else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_MULTI)
                {
                    auto gridGroupBox = static_cast<QGroupBox*>(widget);
                    auto layout = static_cast<QGridLayout*>(gridGroupBox->layout());

                    QSpinBox* spinBox = static_cast<QSpinBox*>(layout->itemAtPosition(0, 1)->widget());

                    std::string json = v.toString().toStdString();

                    std::stringstream lineStream(json);
                    std::string line;
                    for (int j = 1; std::getline(lineStream, line, ';'); j++)
                    {
                        if (j == 1)
                        {
                            LOG_DEBUG("Setting Spinbox Value to {}", std::stoi(line));
                            spinBox->setValue(std::stoi(line));
                            continue;
                        }
                        std::stringstream cellStream(line);
                        std::string cell;
                        for (int k = 0; std::getline(cellStream, cell, '|'); k++)
                        {
                            LOG_DEBUG("Setting List(row={}, col={}) value to {}", j, k, cell);
                            static_cast<QComboBox*>(layout->itemAtPosition(j, k)->widget())->setCurrentText(QString::fromStdString(cell));
                        }
                    }
                }
                else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_MAP_INT)
                    static_cast<QSpinBox*>(widget)->setValue(v.toInt());
            }
        }
    }
}

void NodeModel::restore(QJsonObject const& p)
{
    restoreLayoutItems(_mainLayout, p);
}

QWidget* NodeModel::embeddedWidget()
{
    return _mainWidget;
}
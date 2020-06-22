#include "NodeModel.hpp"

#include <QtWidgets/QLabel>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QComboBox>

#include <tuple>
#include <iostream>
#include <sstream>

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
        if (getParentNode()->nodeState().getEntries(PortType::In).size() > nPorts(PortType::In))
        {
            removeRepeatedConfigGroupBox(inputSpinBox);
        }
        else if (getParentNode()->nodeState().getEntries(PortType::In).size() < nPorts(PortType::In))
        {
            const auto& nodeInfo = nodeManager.registeredNodeTypes().find(_name.toStdString())->second;
            const auto& guiConfigs = nodeInfo.constructorEmpty()->guiConfig();
            for (size_t i = 0; i < guiConfigs.size(); i++)
            {
                const auto& config = guiConfigs.at(i);

                if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_N_INPUT_PORTS)
                {
                    size_t configRepeatedNumber = std::stoul(std::get<std::string>(std::get<3>(config).at(3)));
                    addRepeatedConfigGroupBox(guiConfigs, _mainLayout, nPorts(PortType::In), i, configRepeatedNumber);
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

void NodeModel::addListListIntRow(std::vector<std::variant<std::string, NAV::Node::ConfigOptionsBase>> config, int row, QGridLayout* layout, QGroupBox* gridGroupBox, QFormLayout* formLayout)
{
    LOG_TRACE("called");

    for (size_t j = 0; j < config.size(); j++)
    {
        const std::string& line = std::get<std::string>(config.at(j));
        if (j <= 1)
        {
            QComboBox* comboBox = new QComboBox(gridGroupBox);
            std::stringstream lineStream(line);
            std::string cell;
            while (std::getline(lineStream, cell, '|'))
            {
                if (cell.at(0) == '[')
                {
                    comboBox->addItem(QString::fromStdString(cell.substr(1, cell.size() - 2)));
                    comboBox->setCurrentIndex(comboBox->count() - 1);
                }
                else
                    comboBox->addItem(QString::fromStdString(cell));
            }
            comboBox->setStyleSheet("QComboBox { background-color: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
            comboBox->setProperty("Row", row);
            comboBox->setObjectName("List " + QString::fromStdString(std::to_string(j)) + " in row " + QString::fromStdString(std::to_string(row)));

            layout->addWidget(comboBox, row, static_cast<int>(j));
        }
        else if (j == 2)
        {
            QSpinBox* spinBox = new QSpinBox(gridGroupBox);
            std::stringstream lineStream(line);
            std::string cell;
            for (size_t k = 0; k < 3; k++)
            {
                std::getline(lineStream, cell, '|');
                if (k == 0)
                    spinBox->setMinimum(std::stoi(cell));
                else if (k == 1)
                    spinBox->setValue(std::stoi(cell));
                else if (k == 2)
                    spinBox->setMaximum(std::stoi(cell));
            }
            spinBox->setStyleSheet("QSpinBox { background-color: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
            spinBox->setProperty("Row", row);
            spinBox->setObjectName("SpinBox in row " + QString::fromStdString(std::to_string(row)));

            connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged),
                    [this, config, spinBox, layout, gridGroupBox, formLayout](int i) {
                        if (i != -1 && spinBox->property("Row") == layout->rowCount() - 1)
                            this->addListListIntRow(config, layout->rowCount(), layout, gridGroupBox, formLayout);
                    });

            layout->addWidget(spinBox, row, static_cast<int>(j));
        }
    }
}

void NodeModel::addGuiElementForConfig(const NAV::Node::ConfigOptions& config, const std::vector<NAV::Node::ConfigOptions>& guiConfigs, const size_t configPosition, QFormLayout* _layout, int layoutInsertPosition, QString prefix)
{
    LOG_TRACE("called");

    QString description = QString::fromStdString(std::get<1>(config));

    if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_BOOL)
    {
        QCheckBox* checkBox = new QCheckBox();

        checkBox->setChecked(std::stoi(std::get<std::string>(std::get<3>(config).front())));
        checkBox->setStyleSheet("QCheckBox::indicator:unchecked { border: 1px solid rgb(220,220,220); }");
        _layout->insertRow(layoutInsertPosition, description, checkBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_INT
             || std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_N_INPUT_PORTS)
    {
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
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_STRING)
    {
        QLineEdit* lineEdit = new QLineEdit();

        lineEdit->setStyleSheet("QLineEdit { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
        lineEdit->setText(QString::fromStdString(std::get<std::string>(std::get<3>(config).front())));
        _layout->insertRow(layoutInsertPosition, description, lineEdit);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_LIST)
    {
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

        comboBox->setStyleSheet("QComboBox { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
        _layout->insertRow(layoutInsertPosition, description, comboBox);

        connect(comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
                [this, guiConfigs, configPosition, _layout, layoutInsertPosition]() {
                    this->updateVariants(guiConfigs, configPosition, _layout, layoutInsertPosition);
                });
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_INT)
    {
        QGroupBox* gridGroupBox = new QGroupBox(description);
        QGridLayout* layout = new QGridLayout();

        layout->addWidget(new QLabel("X Data Source", gridGroupBox), 0, 0);
        layout->addWidget(new QLabel("Y Data Source", gridGroupBox), 0, 1);
        layout->addWidget(new QLabel("Window", gridGroupBox), 0, 2);

        addListListIntRow(std::get<3>(config), 1, layout, gridGroupBox, _layout);

        gridGroupBox->setLayout(layout);
        // gridGroupBox->setStyleSheet("QGroupBox { color: green; }");
        _layout->insertRow(layoutInsertPosition, gridGroupBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_MAP_INT)
    {
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

    if (std::get<0>(config) != NAV::Node::ConfigOptionType::CONFIG_VARIANT)
    {
        QWidget* widget = _layout->itemAt(layoutInsertPosition, QFormLayout::ItemRole::FieldRole)->widget();
        widget->setObjectName(prefix + description);
        widget->setProperty("type", std::get<0>(config));
        widget->setToolTip(QString::fromStdString(std::get<2>(config)));
        LOG_DEBUG("Added GUI Element of type={} with Name={}", std::get<0>(config), (prefix + description).toStdString());
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
    // gridGroupBox->setStyleSheet("QGroupBox { color: green; }");
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

void NodeModel::removeRepeatedConfigGroupBox(QSpinBox* inputSpinBox)
{
    LOG_TRACE("called");

    auto portNumber = inputSpinBox->value() + 1;

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

NodeDataType determinePortTypeForLayout(QFormLayout* layout, PortIndex portIndex)
{
    for (int i = 0; i < layout->rowCount(); i++)
    {
        QWidget* widget = layout->itemAt(i, QFormLayout::ItemRole::FieldRole)->widget();

        if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST
            && widget->objectName() == QString::fromStdString(std::to_string(portIndex + 1) + "-Port Type"))
        {
            return { static_cast<QComboBox*>(widget)->currentText(),
                     static_cast<QComboBox*>(widget)->currentText() };
        }
        if (widget->layout())
        {
            auto portTypeFromLayout = determinePortTypeForLayout(static_cast<QFormLayout*>(widget->layout()), portIndex);
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
    const auto& nodeInfo = nodeManager.registeredNodeTypes().find(_name.toStdString())->second;

    uint8_t port = static_cast<uint8_t>(portIndex);

    if (portType == PortType::In)
    {
        auto portTypeFromLayout = determinePortTypeForLayout(_mainLayout, portIndex);
        if (portTypeFromLayout.name != "")
        {
            return portTypeFromLayout;
        }

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

        if (widget->layout() && (widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_INT))
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
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_STRING)
                modelJson[widget->objectName()] = static_cast<QLineEdit*>(widget)->text();
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST)
                modelJson[widget->objectName()] = static_cast<QComboBox*>(widget)->currentText();
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_INT)
            {
                auto gridGroupBox = static_cast<QGroupBox*>(widget);
                auto layout = static_cast<QGridLayout*>(gridGroupBox->layout());
                std::string json;
                for (int j = 1; j < layout->rowCount(); j++)
                {
                    QComboBox* xlist = static_cast<QComboBox*>(layout->itemAtPosition(j, 0)->widget());
                    QComboBox* ylist = static_cast<QComboBox*>(layout->itemAtPosition(j, 1)->widget());
                    QSpinBox* spinBox = static_cast<QSpinBox*>(layout->itemAtPosition(j, 2)->widget());

                    if (spinBox->value() != -1)
                    {
                        if (!json.empty())
                            json += ";";

                        json += xlist->currentText().toStdString() + "|" + ylist->currentText().toStdString() + "|" + std::to_string(spinBox->value());
                    }
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

        if (widget->layout() && (widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_INT))
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
                else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_STRING)
                    static_cast<QLineEdit*>(widget)->setText(v.toString());
                else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST)
                    static_cast<QComboBox*>(widget)->setCurrentText(v.toString());
                else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_INT)
                {
                    auto gridGroupBox = static_cast<QGroupBox*>(widget);
                    auto layout = static_cast<QGridLayout*>(gridGroupBox->layout());

                    std::string json = v.toString().toStdString();

                    std::stringstream lineStream(json);
                    std::string line;
                    for (int j = 1; std::getline(lineStream, line, ';'); j++)
                    {
                        std::stringstream cellStream(line);
                        std::string cell;
                        for (int k = 0; std::getline(cellStream, cell, '|'); k++)
                        {
                            if (k <= 1)
                                static_cast<QComboBox*>(layout->itemAtPosition(j, k)->widget())->setCurrentText(QString::fromStdString(cell));
                            else if (k == 2)
                                static_cast<QSpinBox*>(layout->itemAtPosition(j, k)->widget())->setValue(std::stoi(cell));
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
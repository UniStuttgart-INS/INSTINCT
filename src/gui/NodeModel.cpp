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
                    size_t configRepeatedNumber = std::stoul(std::get<3>(config).at(3));
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

void NodeModel::addListListIntRow(std::vector<std::string> config, int row, QGridLayout* layout, QGroupBox* gridGroupBox, QFormLayout* formLayout)
{
    LOG_TRACE("called");

    for (size_t j = 0; j < config.size(); j++)
    {
        const std::string& line = config.at(j);
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

            connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged),
                    [this, config, spinBox, layout, gridGroupBox, formLayout](int i) {
                        if (i != -1 && spinBox->property("Row") == layout->rowCount() - 1)
                            this->addListListIntRow(config, layout->rowCount(), layout, gridGroupBox, formLayout);
                    });

            layout->addWidget(spinBox, row, static_cast<int>(j));
        }
    }
}

void NodeModel::addGuiElementForConfig(const NAV::Node::ConfigOptions& config, QFormLayout* _layout, QString prefix)
{
    LOG_TRACE("called");

    QString description = QString::fromStdString(std::get<1>(config));

    if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_BOOL)
    {
        widgets.push_back(new QCheckBox());
        QCheckBox* checkBox = static_cast<QCheckBox*>(widgets.at(widgets.size() - 1));

        checkBox->setChecked(std::stoi(std::get<3>(config).front()));
        checkBox->setStyleSheet("QCheckBox::indicator:unchecked { border: 1px solid rgb(220,220,220); }");
        _layout->addRow(description, checkBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_INT
             || std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_N_INPUT_PORTS)
    {
        widgets.push_back(new QSpinBox());
        QSpinBox* spinBox = static_cast<QSpinBox*>(widgets.at(widgets.size() - 1));

        int defaultValue = 0;
        for (size_t j = 0; j < 3; j++)
        {
            std::string cell = std::get<3>(config).at(j);
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
        _layout->addRow(description, spinBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_FLOAT)
    {
        widgets.push_back(new QDoubleSpinBox());
        QDoubleSpinBox* doubleSpinBox = static_cast<QDoubleSpinBox*>(widgets.at(widgets.size() - 1));

        double defaultValue = 0.0;
        for (size_t j = 0; j < 3; j++)
        {
            std::string cell = std::get<3>(config).at(j);
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
        _layout->addRow(description, doubleSpinBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_STRING)
    {
        widgets.push_back(new QLineEdit());
        QLineEdit* lineEdit = static_cast<QLineEdit*>(widgets.at(widgets.size() - 1));

        lineEdit->setStyleSheet("QLineEdit { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
        lineEdit->setText(QString::fromStdString(std::get<3>(config).front()));
        _layout->addRow(description, lineEdit);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_LIST)
    {
        widgets.push_back(new QComboBox());
        QComboBox* comboBox = static_cast<QComboBox*>(widgets.at(widgets.size() - 1));

        for (auto& cell : std::get<3>(config))
        {
            if (cell.at(0) == '[')
            {
                comboBox->addItem(QString::fromStdString(cell.substr(1, cell.size() - 2)));
                comboBox->setCurrentIndex(comboBox->count() - 1);
            }
            else
                comboBox->addItem(QString::fromStdString(cell));
        }

        comboBox->setStyleSheet("QComboBox { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
        _layout->addRow(description, comboBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_INT)
    {
        widgets.push_back(new QGroupBox(description));
        QGroupBox* gridGroupBox = static_cast<QGroupBox*>(widgets.at(widgets.size() - 1));
        QGridLayout* layout = new QGridLayout;

        layout->addWidget(new QLabel("X Data Source", gridGroupBox), 0, 0);
        layout->addWidget(new QLabel("Y Data Source", gridGroupBox), 0, 1);
        layout->addWidget(new QLabel("Window", gridGroupBox), 0, 2);

        addListListIntRow(std::get<3>(config), 1, layout, gridGroupBox, _layout);

        gridGroupBox->setLayout(layout);
        // gridGroupBox->setStyleSheet("QGroupBox { color: green; }");
        _layout->addRow(gridGroupBox);
    }
    else if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_MAP_INT)
    {
        widgets.push_back(new QSpinBox());
        QSpinBox* spinBox = static_cast<QSpinBox*>(widgets.at(widgets.size() - 1));

        int defaultValue = 0;
        for (size_t j = 0; j < 4; j++)
        {
            std::string cell = std::get<3>(config).at(j);
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
        _layout->addRow(description, spinBox);
    }

    QWidget* widget = widgets.at(widgets.size() - 1);
    widget->setObjectName(prefix + description);
    widget->setProperty("type", std::get<0>(config));
    widget->setToolTip(QString::fromStdString(std::get<2>(config)));
}

void NodeModel::addRepeatedConfigGroupBox(const std::vector<NAV::Node::ConfigOptions>& guiConfigs,
                                          QFormLayout* _layout,
                                          size_t portNumber,
                                          size_t configRepeatedStart,
                                          size_t configRepeatedNumber)
{
    LOG_TRACE("called");

    QString description = QString::fromStdString("Port " + std::to_string(portNumber));
    widgets.push_back(new QGroupBox(description));
    QGroupBox* gridGroupBox = static_cast<QGroupBox*>(widgets.at(widgets.size() - 1));
    QFormLayout* layout = new QFormLayout;
    gridGroupBox->setObjectName(description);

    for (size_t k = 0; k < configRepeatedNumber; k++)
    {
        const auto& configR = guiConfigs.at(configRepeatedStart + 1 + k);

        addGuiElementForConfig(configR, layout, QString::fromStdString(std::to_string(portNumber) + "-"));
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
            for (auto iter = widgets.begin(); iter != widgets.end(); iter++)
            {
                if ((*iter)->objectName() == widget->objectName())
                {
                    widgets.erase(iter);
                    break;
                }
            }

            widget->deleteLater();
        }
        layout->removeRow(0);
    }
}

void NodeModel::removeRepeatedConfigGroupBox(QSpinBox* inputSpinBox)
{
    LOG_TRACE("called");

    auto portNumber = inputSpinBox->value() + 1;

    for (auto iter = widgets.begin(); iter != widgets.end(); iter++)
    {
        if ((*iter)->objectName() == QString::fromStdString("Port " + std::to_string(portNumber)))
        {
            QGroupBox* gridGroupBox = static_cast<QGroupBox*>(*iter);

            clearLayout(static_cast<QFormLayout*>(gridGroupBox->layout()));

            for (int i = 0; i < _mainLayout->rowCount(); i++)
            {
                if (_mainLayout->itemAt(i, QFormLayout::ItemRole::FieldRole)->widget()->objectName() == gridGroupBox->objectName())
                {
                    _mainLayout->removeRow(i);
                    break;
                }
            }

            widgets.erase(iter);
            break;
        }
    }
}

NodeModel::NodeModel(QString const& name)
    : _name(name), _mainWidget(new QWidget())
{
    LOG_TRACE("called for {}", name.toStdString());

    _mainWidget->setStyleSheet("QWidget { background-color: transparent; color: white }");
    QFormLayout* _layout = new QFormLayout(_mainWidget);
    _mainLayout = _layout;
    _layout->setContentsMargins(0, 5, 0, 0);
    _layout->setLabelAlignment(Qt::AlignRight);

    const auto& nodeInfo = nodeManager.registeredNodeTypes().find(name.toStdString())->second;
    const auto& guiConfigs = nodeInfo.constructorEmpty()->guiConfig();

    for (size_t i = 0; i < guiConfigs.size(); i++)
    {
        const auto& config = guiConfigs.at(i);

        addGuiElementForConfig(config, _layout);

        if (std::get<0>(config) == NAV::Node::ConfigOptionType::CONFIG_N_INPUT_PORTS)
        {
            size_t configRepeatedNumber = std::stoul(std::get<3>(config).at(3));

            auto minPort = std::stoul(std::get<3>(config).at(0));
            auto nPort = std::stoul(std::get<3>(config).at(1));

            for (size_t j = minPort - 1; j < nPort; j++)
            {
                addRepeatedConfigGroupBox(guiConfigs, _layout, j + 1, i, configRepeatedNumber);
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
        for (auto& widget : widgets)
        {
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

NodeDataType NodeModel::dataType(PortType portType, PortIndex portIndex) const
{
    const auto& nodeInfo = nodeManager.registeredNodeTypes().find(_name.toStdString())->second;

    uint8_t port = static_cast<uint8_t>(portIndex);

    if (portType == PortType::In)
    {
        for (auto& widget : widgets)
        {
            if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST
                && widget->objectName() == QString::fromStdString(std::to_string(portIndex + 1) + "-Port Type"))
            {
                return { static_cast<QComboBox*>(widget)->currentText(),
                         static_cast<QComboBox*>(widget)->currentText() };
            }
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

QJsonObject NodeModel::save() const
{
    QJsonObject modelJson = NodeDataModel::save();

    for (auto& widget : widgets)
    {
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

    return modelJson;
}

void NodeModel::restore(QJsonObject const& p)
{
    for (auto& widget : widgets)
    {
        if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_N_INPUT_PORTS)
        {
            static_cast<QSpinBox*>(widget)->setValue(p["nInputPorts"].toInt());
            getParentNode()->nodeState().getEntries(PortType::In).resize(static_cast<size_t>(p["nInputPorts"].toInt()));
            break;
        }
    }

    for (auto& widget : widgets)
    {
        // std::cout << "Restoring " << widget->objectName().toStdString() << std::endl;

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

QWidget* NodeModel::embeddedWidget()
{
    return _mainWidget;
}
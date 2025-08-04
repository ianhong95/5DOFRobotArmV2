#ifndef POSITIONTABLEMODEL_H
#define POSITIONTABLEMODEL_H

#include <QAbstractTableModel>
#include <QVector>
#include <QString>

#include <string>

/* This class is to be used as a controller for the position TableView. It inherits from
 * QAbstractTableModel (an abstract class, cannot be instantiated directly, must be subclassed)
 * because QTableView expects that interface. This class REQUIRES 4 "magic" methods to
 * tell Qt about the table, otherwise you can't instantiate it normally. Implementing the "magic"
 * methods makes it not an abstract class anymore.
 *
 * We need to tell Qt WHAT to count when counting rows and columns because that data is not
 * known to Qt. Hence, we override the virtual methods with our specific application.
 *
 * QModelIndex represents a specific cell in the table. Use row() and column() to target cell.
 */

struct SavedPosition {
    int index;
    QString alias;
    float x, y, z;
};

class PositionTableModel : public QAbstractTableModel {
    Q_OBJECT
public:
    // This constructor makes it so PositionModel accepts a parent parameter ("this" in this case).
    PositionTableModel(QObject *parent = nullptr);

    /* =============
     * MAGIC METHODS
     * ============= */

    // 4 REQUIRED "magic" methods that are based on the virtual methods in QAbstractItemModel

    // QModelIndex() returns an invalid index. This just means the default value is invalid ("null").
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;

    /* By default, the data's role is DisplayRole, which means it's to be rendered as text.
     * https://doc.qt.io/qt-6/qt.html#ItemDataRole-enum
     * This method returns the data stored under the given role for the item given by index. */
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

    /* Orientation is either Qt::Horizontal or Qt::Vertical
     * For horizontal headers, the section number corresponds to the column number.
     * This method returns the table headers as strings. */
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

    /* ===============
     * REGULAR METHODS
     * =============== */

    void addRow(const SavedPosition &positionData);

private:
    QVector<SavedPosition> savedPositions;
    Qt::ItemFlags flags(const QModelIndex &index) const override;

    /* This method needs to be implemented to update the table automatically upon editing without any
     * signals/slots. Otherwise, the cell value will just revert after you edit. */
    bool setData(const QModelIndex &index, const QVariant &value, int role) override;
};

#endif // POSITIONTABLEMODEL_H

# from PyQt5.QtWidgets import QTableWidget, QTreeView, QFormLayout, QLabel, QApplication, QTableWidgetItem
# from PyQt5 import QtWidgets
# from PyQt5.QtGui import QStandardItemModel, QStandardItem
import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.Qt import Qt

class QHLine(QFrame):
    def __init__(self):
        super(QHLine, self).__init__()
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)


class QVLine(QFrame):
    def __init__(self):
        super(QVLine, self).__init__()
        self.setFrameShape(QFrame.VLine)
        self.setFrameShadow(QFrame.Sunken)

class MyTableWidget(QTableWidget):
    def __init__(self, parent=None):
        super(MyTableWidget, self).__init__(parent)

        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.viewport().setAcceptDrops(True)
        self.setDragDropOverwriteMode(False)
        self.setDropIndicatorShown(True)

        self.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.setDragDropMode(QAbstractItemView.InternalMove)

        self.item_cnt = 0

    def keyPressEvent(self, e: QKeyEvent) -> None:
        super().keyPressEvent(e)
        if e.key() == Qt.Key_Delete:
            rows = []
            for i in (self.selectedIndexes()):
                rows.append(i.row())
            rows = list(set(rows))
            for i in reversed(rows):
                self.removeRow(i)
    
    def dragEnterEvent(self, e):
        if e.mouseButtons() == Qt.LeftButton:
            # print("dragEnterEvent")
            e.accept()

    def dropEvent(self, event: QDropEvent):
        if not event.isAccepted() and event.source() == self:
            drop_row = self.drop_on(event)

            rows = sorted(set(item.row() for item in self.selectedItems()))
            rows_to_move = [[QTableWidgetItem(self.item(row_index, column_index)) for column_index in range(self.columnCount())]
                            for row_index in rows]
            for row_index in reversed(rows):
                self.removeRow(row_index)
                if row_index < drop_row:
                    drop_row -= 1

            for row_index, data in enumerate(rows_to_move):
                row_index += drop_row
                self.insertRow(row_index)
                for column_index, column_data in enumerate(data):
                    self.setItem(row_index, column_index, column_data)
                    # self.setItem(row_index, column_index+1, QTableWidgetItem(str(self.item_cnt)))
                    # self.item_cnt += 1
            event.accept()
            for row_index in range(len(rows_to_move)):
                self.item(drop_row + row_index, 0).setSelected(True)
                self.item(drop_row + row_index, 1).setSelected(True)
        # super().dropEvent(event)
        else:
            n_row = self.drop_on(event)
            # print(n_row)
            self.insertRow(n_row)
            self.setItem(n_row, 0, QTableWidgetItem(event.source().currentItem().text(0)))
            self.setItem(n_row, 1, QTableWidgetItem(str(self.item_cnt)))
            # self.item_cnt += 1

    def drop_on(self, event):
        index = self.indexAt(event.pos())
        if not index.isValid():
            return self.rowCount()

        return index.row() + 1 if self.is_below(event.pos(), index) else index.row()

    def is_below(self, pos, index):
        rect = self.visualRect(index)
        margin = 2
        if pos.y() - rect.top() < margin:
            return False
        elif rect.bottom() - pos.y() < margin:
            return True
        # noinspection PyTypeChecker
        return rect.contains(pos, True) and not (int(self.model().flags(index)) & Qt.ItemIsDropEnabled) and pos.y() >= rect.center().y()
    
    # def dragLeaveEvent(self, e):
    #     print("dragLeaveEvent")


class MyTreeWidget(QTreeWidget):
    def __init__(self, parent=None):
        super(MyTreeWidget, self).__init__(parent)
        self.setDragEnabled(True)

    def startDrag(self, actions):
        if self.currentItem().parent() == None:
            print("can't drag")
        else:
            super().startDrag(actions)
    
    def fillItems(self, item, value : dict):
        if type(value) is dict:
            for key, val in value.items():
                child = QTreeWidgetItem(item)
                child.setText(0, key)
                self.fillItems(child, val)

if __name__ == "__main__":
    class MyWindow(QDialog):
        def __init__(self, *args, obj=None, **kwargs):
            super(MyWindow, self).__init__(*args, **kwargs)
            self.initUi()

        def initUi(self):
            self.table = MyTableWidget()
            self.table.setMinimumHeight(1000)
            self.tree = MyTreeWidget()
            self.tree.setMinimumWidth(300)

            lo = QFormLayout()
            lo.addRow(QLabel("Let's find out how to use that Drag&Drop"))
            lo.addRow(self.tree, self.table)
            self.setLayout(lo)
            self.setWindowTitle('Drag and Drop Example')
            self.setGeometry(1800, 700, 800, 1000)

            # fill the treeview
            self.tree.setDragDropMode(QAbstractItemView.DragOnly)
            self.tree.setColumnCount(1)
            parents_cnt = 0

            # for i in range(5):
            #     parent = QTreeWidgetItem(self.tree)
            #     parent.setText(0, f'parent {i+1}')
            #     for i in range(6):
            #         child = QTreeWidgetItem(parent)
            #         child.setText(0, f'child {i+1}')

            dict = {'parent1' : 
                        {'child1': 1,
                        'child2': 2,
                        'child3': 3,
                        'child4': 4,
                        'child5': 5},
                    'parent2' : 
                        {'child6': 6,
                        'child7': 7,
                        'child8': 8,
                        'child9': 9,
                        'child10': 10},
                    'parent3' : 
                        {'child11': 11,
                        'child12': 12,
                        'child13': 13,
                        'child14': 14,
                        'child15': 15}}
            self.tree.fillItems(self.tree, dict)
            
            
            for i in range(self.tree.topLevelItemCount()):
                print(self.tree.topLevelItem(i).text(0)) 
            

            # my table
            self.table.setColumnCount(2)
            self.table.setColumnWidth(0, 250)
            self.table.setColumnWidth(1, 150)
            self.table.setHorizontalHeaderLabels(['Name', 'Value'])
            


    app = QApplication(sys.argv)
    ex = MyWindow()
    ex.show()
    app.exec()


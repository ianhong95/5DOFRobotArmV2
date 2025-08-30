"""Serverless SQL database for storing robot positions."""

import sqlite3

class SavedPositionsDB:
    TABLE_NAME = "saved_positions"
    TABLE_COLUMNS = [
        "id INTEGER PRIMARY KEY AUTOINCREMENT",
        "alias TEXT",
        "x FLOAT",
        "y FLOAT",
        "z FLOAT",
        "j1_angle FLOAT",
        "j2_angle FLOAT",
        "j3_angle FLOAT",
        "j4_angle FLOAT",
        "j5_angle FLOAT",
    ]

    TABLE_COLUMN_NAMES = ["alias", "x", "y", "z", "j1_angle", "j2_angle", "j3_angle", "j4_angle", "j5_angle"]

    def __init__(self):
        self.connection = sqlite3.connect("saved_positions.db")
        self.cursor = self.connection.cursor()

        self._initialize_table()

        print("Database initialized.")

    def _initialize_table(self):
        table_params = ", ".join(self.TABLE_COLUMNS)
        self.cursor.execute(f"CREATE TABLE IF NOT EXISTS {self.TABLE_NAME} ({table_params})")

    def add_row(self, values: list) -> int:
        """Add a row (save a position entry) to the table. Returns an integer for the id (autoincremented) of the row."""
        column_names = ", ".join(self.TABLE_COLUMN_NAMES)

        self.cursor.execute(f"INSERT INTO {self.TABLE_NAME} ({column_names}) VALUES ('', {', '.join(str(value) for value in values)})")
        self.connection.commit()

        return self.cursor.lastrowid

    def get_coords_from_idx(self, id: int) -> tuple:
        result_row = self.cursor.execute(f"SELECT x, y, z FROM {self.TABLE_NAME} WHERE id = {id}")
        coordinates = result_row.fetchone()

        return coordinates
    
    def get_joint_angles_from_idx(self, id: int) -> tuple:
        result_row = self.cursor.execute(f"SELECT j1_angle, j2_angle, j3_angle, j4_angle, j5_angle FROM {self.TABLE_NAME} WHERE id = {id}")
        joint_angles = result_row.fetchone()

        return joint_angles
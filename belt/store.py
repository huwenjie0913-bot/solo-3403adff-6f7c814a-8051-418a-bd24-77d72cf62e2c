"""SQLite 方案存储。"""
import json
import os
import sqlite3
import time

DB_ENV = "BELT_DB"
DEFAULT_DB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "belt.db")


def db_path():
    return os.environ.get(DB_ENV, DEFAULT_DB)


def connect():
    conn = sqlite3.connect(db_path())
    conn.row_factory = sqlite3.Row
    return conn


def init_db():
    with connect() as conn:
        conn.execute(
            """CREATE TABLE IF NOT EXISTS schemes (
                   id INTEGER PRIMARY KEY AUTOINCREMENT,
                   name TEXT NOT NULL,
                   data TEXT NOT NULL,
                   version INTEGER NOT NULL DEFAULT 1,
                   created_at REAL NOT NULL,
                   updated_at REAL NOT NULL
               )""")
        # 旧库平滑升级
        cols = [r[1] for r in conn.execute("PRAGMA table_info(schemes)").fetchall()]
        if "version" not in cols:
            conn.execute("ALTER TABLE schemes ADD COLUMN version INTEGER NOT NULL DEFAULT 1")
        conn.commit()


def list_schemes():
    with connect() as conn:
        rows = conn.execute(
            "SELECT id, name, version, created_at, updated_at FROM schemes "
            "ORDER BY updated_at DESC"
        ).fetchall()
        return [dict(r) for r in rows]


def get_scheme(scheme_id):
    with connect() as conn:
        row = conn.execute("SELECT * FROM schemes WHERE id=?", (scheme_id,)).fetchone()
    if not row:
        return None
    d = dict(row)
    d["data"] = json.loads(d["data"])
    return d


def create_scheme(name, data):
    now = time.time()
    with connect() as conn:
        cur = conn.execute(
            "INSERT INTO schemes(name, data, version, created_at, updated_at) "
            "VALUES(?,?,?,?,?)",
            (name, json.dumps(data, ensure_ascii=False), 1, now, now))
        conn.commit()
        return cur.lastrowid


def update_scheme(scheme_id, name, data):
    now = time.time()
    with connect() as conn:
        cur = conn.execute(
            "UPDATE schemes SET name=?, data=?, "
            "version=COALESCE(version,1)+1, updated_at=? WHERE id=?",
            (name, json.dumps(data, ensure_ascii=False), now, scheme_id))
        conn.commit()
        return cur.rowcount > 0


def delete_scheme(scheme_id):
    with connect() as conn:
        cur = conn.execute("DELETE FROM schemes WHERE id=?", (scheme_id,))
        conn.commit()
        return cur.rowcount > 0

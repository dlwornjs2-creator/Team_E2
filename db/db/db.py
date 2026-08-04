"""DB 노드 — 4단계.

순서도 대응:
  부팅 -> DB 연결 -> (실패 시 재시도) -> 대기
       -> 상태 노드 초기화 요청 -> 응답 -> 대기
       -> 요청 분기: 데이터 저장 / DB 탐색

제공 서비스
  /db/init      interfaces/NodeInit  상태 노드의 초기화 확인 요청 (전 노드 공용 타입)
  /db/save      interfaces/DbSave    작업 완료 후 일괄 저장
  /db/load      interfaces/DbLoad    물건 유무 및 위치 조회
  /db/clear     std_srvs/Trigger     전체 삭제 (테스트용, allow_clear 로 차단 가능)

JSON 형식
  NodeInit 요청  {"node": "state_node"}
  NodeInit 응답  {"ready": true, "db_path": "...", "items": 4, "sightings": 9}

  DbSave 요청  {"source": "item_node",
               "items": [{"name": "컵", "class_label": "cup",
                          "location": "주방 싱크대", "confidence": 0.92}, ...]}
  DbSave 응답  {"inserted": 1, "updated": 2, "results": [...]}

  DbLoad 요청  {}                      -> 전체
               {"name": "컵"}          -> 이름으로 조회
               {"class_label": "cup"}  -> 클래스로 조회
  DbLoad 응답  {"count": 4, "items": [{...}, ...]}
"""

import json
import os
import sqlite3
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from interfaces.srv import DbLoad, DbSave, NodeInit


# ----------------------------------------------------------------------
# 스키마
# ----------------------------------------------------------------------
CREATE_ITEMS = """
CREATE TABLE IF NOT EXISTS items (
    id          INTEGER PRIMARY KEY,
    name        TEXT NOT NULL UNIQUE COLLATE NOCASE,  -- 사람이 부르는 이름
    class_label TEXT NOT NULL,                        -- 검출기가 뱉는 클래스명
    location    TEXT NOT NULL,                        -- 마지막으로 본 위치
    last_seen   TEXT NOT NULL                         -- ISO8601 문자열
)
"""

CREATE_SIGHTINGS = """
CREATE TABLE IF NOT EXISTS sightings (
    id          INTEGER PRIMARY KEY,
    item_id     INTEGER REFERENCES items(id) ON DELETE SET NULL,
    name        TEXT NOT NULL,      -- 그 시점의 값을 복사 저장
    class_label TEXT NOT NULL,
    location    TEXT NOT NULL,
    seen_at     TEXT NOT NULL,
    confidence  REAL,               -- 인식 신뢰도
    source      TEXT                -- 어느 노드가 보고했는지
)
"""

CREATE_INDEXES = [
    "CREATE INDEX IF NOT EXISTS idx_items_class ON items(class_label)",
    "CREATE INDEX IF NOT EXISTS idx_sightings_name ON sightings(name, seen_at)",
]


# ----------------------------------------------------------------------
# DB 조작 (ROS 와 무관한 순수 함수)
# ----------------------------------------------------------------------
def now_iso() -> str:
    return datetime.now().isoformat(timespec='seconds')


def normalize_record(rec):
    """(name, class_label, location, confidence) 튜플로 통일한다."""
    if isinstance(rec, dict):
        name = rec.get('name')
        label = rec.get('class_label', '')
        location = rec.get('location')
        conf = rec.get('confidence')
    else:
        name, label, location, conf = rec

    if not name or not str(name).strip():
        raise ValueError('name 이 비어 있습니다')
    if not location or not str(location).strip():
        raise ValueError(f"'{name}' 의 location 이 비어 있습니다")

    return (str(name).strip(), str(label), str(location).strip(),
            float(conf) if conf is not None else None)


def _write_one(cur, name, class_label, location, confidence, source, seen_at):
    """목격 1건을 커서에 기록한다. (트랜잭션은 호출한 쪽이 연다)"""
    before = cur.execute(
        'SELECT id FROM items WHERE name = ?', (name,)).fetchone()

    cur.execute(
        """
        INSERT INTO items (name, class_label, location, last_seen)
        VALUES (?, ?, ?, ?)
        ON CONFLICT(name) DO UPDATE SET
            class_label = excluded.class_label,
            location    = excluded.location,
            last_seen   = excluded.last_seen
        """,
        (name, class_label, location, seen_at),
    )

    item_id = cur.execute(
        'SELECT id FROM items WHERE name = ?', (name,)).fetchone()[0]

    cur.execute(
        """
        INSERT INTO sightings
            (item_id, name, class_label, location, seen_at, confidence, source)
        VALUES (?, ?, ?, ?, ?, ?, ?)
        """,
        (item_id, name, class_label, location, seen_at, confidence, source),
    )

    return item_id, ('update' if before else 'insert')


def record_sightings_batch(conn, records, source=''):
    """작업 1회분을 한 트랜잭션으로 기록한다.

    본 것만 갱신(UPSERT)하고, 이번에 못 본 물건은 건드리지 않는다.
    하나라도 실패하면 전체가 롤백된다.
    """
    seen_at = now_iso()          # 같은 작업분은 같은 시각으로 묶는다
    normalized = [normalize_record(r) for r in records]
    results = []

    with conn:                   # 여기서 트랜잭션 한 번만 열린다
        cur = conn.cursor()
        for name, label, location, conf in normalized:
            item_id, action = _write_one(
                cur, name, label, location, conf, source, seen_at)
            results.append({'name': name, 'id': item_id, 'action': action})

    return results


def search_items(conn, name=None, class_label=None):
    """조건에 맞는 items 를 dict 리스트로 돌려준다. 조건이 없으면 전체."""
    conn.row_factory = sqlite3.Row

    if name:
        sql = 'SELECT * FROM items WHERE name = ? ORDER BY name'
        args = (name.strip(),)
    elif class_label:
        sql = 'SELECT * FROM items WHERE class_label = ? ORDER BY name'
        args = (class_label.strip(),)
    else:
        sql = 'SELECT * FROM items ORDER BY id'
        args = ()

    return [dict(row) for row in conn.execute(sql, args)]


def clear_all(conn):
    """items / sightings 를 통째로 비운다. 되돌릴 수 없다.

    반환: (지워진 items 수, 지워진 sightings 수)
    """
    with conn:
        cur = conn.cursor()
        n_items = cur.execute('SELECT COUNT(*) FROM items').fetchone()[0]
        n_sight = cur.execute('SELECT COUNT(*) FROM sightings').fetchone()[0]
        cur.execute('DELETE FROM sightings')   # 자식 먼저
        cur.execute('DELETE FROM items')
    return n_items, n_sight


# ----------------------------------------------------------------------
# 서비스 요청 처리 (JSON in -> JSON out)
# ROS 객체를 안 써서 노드 없이도 단독 테스트가 된다
# ----------------------------------------------------------------------
def db_status(conn, db_path):
    """DB 준비 상태를 dict 로 돌려준다."""
    tables = [r[0] for r in conn.execute(
        "SELECT name FROM sqlite_master WHERE type='table' ORDER BY name")]
    return {
        'ready': 'items' in tables and 'sightings' in tables,
        'db_path': db_path,
        'tables': tables,
        'items': conn.execute('SELECT COUNT(*) FROM items').fetchone()[0],
        'sightings': conn.execute('SELECT COUNT(*) FROM sightings').fetchone()[0],
    }


def handle_init(conn, request_json, db_path):
    """초기화 확인 요청을 처리한다. 반환: (success, response_json, message)"""
    try:
        payload = json.loads(request_json) if request_json.strip() else {}
    except json.JSONDecodeError as e:
        return False, '{}', f'JSON 파싱 실패: {e}'

    who = payload.get('node', 'unknown')

    try:
        status = db_status(conn, db_path)
    except sqlite3.Error as e:
        return False, '{}', f'상태 확인 실패: {e}'

    msg = (f"DB 준비됨 (items {status['items']}건, "
           f"sightings {status['sightings']}건) / 요청자={who}")
    return (status['ready'],
            json.dumps(status, ensure_ascii=False),
            msg if status['ready'] else '테이블이 준비되지 않았습니다')


def handle_save(conn, request_json):
    """반환: (success, response_json, message)"""
    try:
        payload = json.loads(request_json) if request_json.strip() else {}
    except json.JSONDecodeError as e:
        return False, '{}', f'JSON 파싱 실패: {e}'

    records = payload.get('items', [])
    source = payload.get('source', 'unknown')

    if not records:
        return False, '{}', '저장할 항목이 없습니다'

    try:
        results = record_sightings_batch(conn, records, source=source)
    except Exception as e:      # noqa: BLE001
        return False, '{}', f'저장 실패 — 전체 롤백: {e}'

    inserted = sum(1 for r in results if r['action'] == 'insert')
    updated = len(results) - inserted
    response = {'inserted': inserted, 'updated': updated, 'results': results}

    return (True, json.dumps(response, ensure_ascii=False),
            f'저장 완료: 신규 {inserted} / 갱신 {updated}')


def handle_search(conn, request_json):
    """반환: (success, response_json, message)"""
    try:
        payload = json.loads(request_json) if request_json.strip() else {}
    except json.JSONDecodeError as e:
        return False, '{}', f'JSON 파싱 실패: {e}'

    name = payload.get('name')
    class_label = payload.get('class_label')

    try:
        rows = search_items(conn, name=name, class_label=class_label)
    except sqlite3.Error as e:
        return False, '{}', f'조회 실패: {e}'

    if name:
        msg = f"'{name}' {'찾음' if rows else '없음'}"
    elif class_label:
        msg = f"class='{class_label}' {len(rows)}건"
    else:
        msg = f'전체 {len(rows)}건'

    # 조회 자체는 성공. 물건이 없는 것은 실패가 아니라 count=0 으로 알린다
    response = {'count': len(rows), 'items': rows}
    return True, json.dumps(response, ensure_ascii=False), msg


class DBNode(Node):

    def __init__(self):
        super().__init__('db_node')

        self.declare_parameter('db_path', '~/.ros/robot_db/robot.db')
        self.declare_parameter('require_init', False)
        self.declare_parameter('allow_clear', True)   # 실서비스에서는 false 로

        raw_path = self.get_parameter('db_path').value
        self.db_path = os.path.abspath(os.path.expanduser(raw_path))
        self.require_init = self.get_parameter('require_init').value
        self.allow_clear = self.get_parameter('allow_clear').value

        self.conn = None
        self.initialized = False
        self.setup_database()

        self.create_service(NodeInit, 'db/init', self.on_init)
        self.create_service(DbSave, 'db/save', self.on_save)
        self.create_service(DbLoad, 'db/load', self.on_load)
        self.create_service(Trigger, 'db/clear', self.on_clear)

        self.dump_tables()
        self.get_logger().info('DB 준비 완료 — 요청 대기 중')

    # ------------------------------------------------------------------
    def setup_database(self):
        """DB 파일을 열고 테이블을 만든다."""
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self.get_logger().info(f'DB 파일 경로: {self.db_path}')

        try:
            self.conn = sqlite3.connect(self.db_path)
            self.conn.execute('PRAGMA foreign_keys = ON')

            cur = self.conn.cursor()
            cur.execute(CREATE_ITEMS)
            cur.execute(CREATE_SIGHTINGS)
            for sql in CREATE_INDEXES:
                cur.execute(sql)
            self.conn.commit()

        except sqlite3.Error as e:
            self.get_logger().fatal(f'DB 초기화 실패: {e}')
            raise

        tables = [row[0] for row in self.conn.execute(
            "SELECT name FROM sqlite_master WHERE type='table' ORDER BY name")]
        self.get_logger().info(f'테이블 확인: {tables}')

    # ------------------------------------------------------------------
    def on_init(self, request, response):
        """상태 노드의 초기화 확인 요청에 응답한다."""
        success, data, message = handle_init(
            self.conn, request.request, self.db_path)

        response.success = success
        response.response = data
        response.message = message

        # 준비가 확인된 경우에만 초기화 완료로 표시한다
        if success:
            self.initialized = True

        log = self.get_logger().info if success else self.get_logger().error
        log(f'[init] {message}')
        return response

    def _check_init(self, response):
        """require_init 이 켜져 있으면 초기화 전 요청을 막는다."""
        if self.require_init and not self.initialized:
            response.success = False
            response.response = '{}'
            response.message = '아직 초기화되지 않았습니다 (/db/init 먼저 호출)'
            self.get_logger().warn(response.message)
            return False
        return True

    # ------------------------------------------------------------------
    def on_save(self, request, response):
        """데이터 저장 — 작업 1회분을 통째로 반영한다."""
        if not self._check_init(response):
            return response

        success, data, message = handle_save(self.conn, request.request)
        response.success = success
        response.response = data
        response.message = message

        log = self.get_logger().info if success else self.get_logger().error
        log(f'[save] {message}')
        if success:
            self.dump_tables()
        return response

    def on_load(self, request, response):
        """DB 탐색 — 물건 유무 및 위치를 돌려준다."""
        if not self._check_init(response):
            return response

        success, data, message = handle_search(self.conn, request.request)
        response.success = success
        response.response = data
        response.message = message

        log = self.get_logger().info if success else self.get_logger().error
        log(f'[load] {message}')
        return response

    def on_clear(self, request, response):
        """전체 삭제 — 테스트용. 되돌릴 수 없다."""
        if not self.allow_clear:
            response.success = False
            response.message = '전체 삭제가 비활성화되어 있습니다 (allow_clear=false)'
            self.get_logger().warn(f'[clear] 거절 — {response.message}')
            return response

        try:
            n_items, n_sight = clear_all(self.conn)
        except sqlite3.Error as e:
            response.success = False
            response.message = f'삭제 실패 — 롤백: {e}'
            self.get_logger().error(f'[clear] {response.message}')
            return response

        response.success = True
        response.message = f'전체 삭제 완료 (items {n_items}건, sightings {n_sight}건)'
        self.get_logger().warn(f'[clear] {response.message}')
        self.dump_tables()
        return response

    # ------------------------------------------------------------------
    def dump_tables(self):
        """현재 DB 내용을 로그로 출력한다. (확인용)"""
        self.conn.row_factory = sqlite3.Row

        items = self.conn.execute('SELECT * FROM items ORDER BY id').fetchall()
        n_sight = self.conn.execute(
            'SELECT COUNT(*) FROM sightings').fetchone()[0]

        self.get_logger().info(
            f'--- items {len(items)}건 / sightings {n_sight}건 ---')
        for r in items:
            self.get_logger().info(
                f"  [{r['id']}] {r['name']} ({r['class_label']}) "
                f"@ {r['location']} / {r['last_seen']}")

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.conn is not None:
            self.conn.close()
            self.get_logger().info('DB 연결 종료')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DBNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:      # noqa: BLE001
        print(f'db_node 종료: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
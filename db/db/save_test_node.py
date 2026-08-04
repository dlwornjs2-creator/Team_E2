"""DB 테스트 노드 (대화형).

터미널에서 숫자를 입력해 DB 노드의 서비스를 호출한다.

  ros2 run db test_node

  1) 테스트 데이터 생성   -> /db/save
  2) 전체 목록 출력       -> /db/load  {}
  3) 이름으로 검색        -> /db/load  {"name": "컵"}
  4) 클래스로 검색        -> /db/load  {"class_label": "cup"}
  5) 직접 등록            -> /db/save  (입력받은 1건)
  8) 전체 삭제            -> /db/clear (되돌릴 수 없음)
  9) 초기화 확인          -> /db/init
  0) 종료
"""

import json

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from interfaces.srv import DbLoad, DbSave, NodeInit


# 탐색 작업 1회분 = 리스트 1개. 호출할 때마다 다음 회차가 나간다
TEST_RUNS = [
    # 1회차 — 3개 신규 등록
    [
        {'name': '컵',     'class_label': 'cup',     'location': '주방 싱크대', 'confidence': 0.92},
        {'name': '리모컨', 'class_label': 'remote',  'location': '거실 소파',   'confidence': 0.88},
        {'name': '안경',   'class_label': 'glasses', 'location': '침실 책상',   'confidence': 0.75},
    ],
    # 2회차 — 컵은 이동(갱신), 책은 신규, 리모컨은 못 봄(그대로 유지)
    [
        {'name': '컵', 'class_label': 'cup',  'location': '거실 테이블', 'confidence': 0.81},
        {'name': '책', 'class_label': 'book', 'location': '침실 책상',   'confidence': 0.69},
    ],
    # 3회차 — 같은 이름이 배치 안에 두 번. 뒤에 것이 최종 위치가 된다
    [
        {'name': '컵', 'class_label': 'cup', 'location': '식탁',   'confidence': 0.70},
        {'name': '컵', 'class_label': 'cup', 'location': '베란다', 'confidence': 0.85},
    ],
]

MENU = """
==================================
  1) 테스트 데이터 생성 (다음 회차)
  2) 전체 목록 출력
  3) 이름으로 검색
  4) 클래스로 검색
  5) 직접 등록
  8) 전체 삭제 (되돌릴 수 없음)
  9) 초기화 확인 (/db/init)
  0) 종료
==================================
선택: """


class DBTestNode(Node):

    def __init__(self):
        super().__init__('db_test_node')

        self.save_cli = self.create_client(DbSave, 'db/save')
        self.load_cli = self.create_client(DbLoad, 'db/load')
        self.init_cli = self.create_client(NodeInit, 'db/init')
        self.clear_cli = self.create_client(Trigger, 'db/clear')

        self.run_index = 0      # 다음에 보낼 테스트 회차

        self.get_logger().info('DB 노드 기다리는 중...')
        for cli, name in ((self.save_cli, 'save'),
                          (self.load_cli, 'load'),
                          (self.init_cli, 'init'),
                          (self.clear_cli, 'clear')):
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'/db/{name} 서비스를 찾을 수 없습니다')
        self.get_logger().info('연결 완료')

    # ------------------------------------------------------------------
    def call(self, client, request):
        """서비스를 호출하고 응답이 올 때까지 기다린다."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            print('  !! 응답 시간 초과')
            return None
        return future.result()

    # ------------------------------------------------------------------
    def send_save(self, items, source):
        req = DbSave.Request()
        req.request = json.dumps(
            {'source': source, 'items': items}, ensure_ascii=False)

        res = self.call(self.save_cli, req)
        if res is None:
            return

        print(f'  [{"OK" if res.success else "실패"}] {res.message}')
        if res.success:
            data = json.loads(res.response)
            for r in data['results']:
                mark = '신규' if r['action'] == 'insert' else '갱신'
                print(f'    {mark}  id={r["id"]}  {r["name"]}')

    def send_load(self, payload, title):
        req = DbLoad.Request()
        req.request = json.dumps(payload, ensure_ascii=False)

        res = self.call(self.load_cli, req)
        if res is None:
            return

        print(f'  [{"OK" if res.success else "실패"}] {res.message}')
        if not res.success:
            return

        data = json.loads(res.response)
        print(f'  --- {title} ({data["count"]}건) ---')
        if data['count'] == 0:
            print('    (없음)')
            return
        for it in data['items']:
            print(f'    [{it["id"]}] {it["name"]} ({it["class_label"]}) '
                  f'@ {it["location"]}  / {it["last_seen"]}')

    # ------------------------------------------------------------------
    def menu_generate(self):
        if self.run_index >= len(TEST_RUNS):
            self.run_index = 0
            print('  (마지막 회차까지 다 보내서 1회차로 되돌아갑니다)')

        items = TEST_RUNS[self.run_index]
        label = self.run_index + 1
        print(f'  {label}회차 {len(items)}건 전송: '
              + ', '.join(f'{i["name"]}@{i["location"]}' for i in items))
        self.send_save(items, source=f'test_run{label}')
        self.run_index += 1

    def menu_manual(self):
        name = input('  이름: ').strip()
        label = input('  클래스명: ').strip()
        location = input('  위치: ').strip()
        if not name or not location:
            print('  !! 이름과 위치는 필수입니다')
            return
        self.send_save(
            [{'name': name, 'class_label': label, 'location': location,
              'confidence': 1.0}],
            source='manual')

    def menu_clear(self):
        print('  !! items / sightings 를 전부 지웁니다. 되돌릴 수 없습니다.')
        answer = input('  정말 지우려면 DELETE 를 그대로 입력하세요: ').strip()
        if answer != 'DELETE':
            print('  취소했습니다')
            return

        res = self.call(self.clear_cli, Trigger.Request())
        if res is not None:
            print(f'  [{"OK" if res.success else "실패"}] {res.message}')
            if res.success:
                self.run_index = 0      # 테스트 회차도 1회차부터 다시
                print('  테스트 회차를 1회차로 되돌렸습니다')

    def menu_init(self):
        req = NodeInit.Request()
        req.request = json.dumps({'node': 'db_test_node'}, ensure_ascii=False)

        res = self.call(self.init_cli, req)
        if res is None:
            return

        print(f'  [{"OK" if res.success else "준비 안 됨"}] {res.message}')
        st = json.loads(res.response) if res.response else {}
        if st:
            print(f"    ready     : {st.get('ready')}")
            print(f"    db_path   : {st.get('db_path')}")
            print(f"    tables    : {st.get('tables')}")
            print(f"    items     : {st.get('items')}건")
            print(f"    sightings : {st.get('sightings')}건")

    # ------------------------------------------------------------------
    def loop(self):
        while rclpy.ok():
            try:
                choice = input(MENU).strip()
            except EOFError:
                break

            if choice == '1':
                self.menu_generate()
            elif choice == '2':
                self.send_load({}, '전체 목록')
            elif choice == '3':
                name = input('  검색할 이름: ').strip()
                self.send_load({'name': name}, f"'{name}' 검색 결과")
            elif choice == '4':
                label = input('  검색할 클래스명: ').strip()
                self.send_load({'class_label': label}, f"'{label}' 검색 결과")
            elif choice == '5':
                self.menu_manual()
            elif choice == '8':
                self.menu_clear()
            elif choice == '9':
                self.menu_init()
            elif choice == '0':
                print('종료합니다')
                break
            else:
                print('  !! 0~5, 8, 9 중에서 선택하세요')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DBTestNode()
        node.loop()
    except KeyboardInterrupt:
        pass
    except Exception as e:      # noqa: BLE001
        print(f'test_node 종료: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
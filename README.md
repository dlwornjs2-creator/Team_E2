# Team_E2
Rokey Boot Camp 
협동 1 - 2 프로젝트 레포지토리

로봇 팔이 식기(Bowl)를 파지하여 원통형 수세미 타워에서 외부 및 내부를 깨끗이 닦아낸 후, 건조 거치대에 안전하게 적재하는 매니퓰레이션 전체 공정을 담고 있습니다. 
실행 완성도와 유지보수성을 높이기 위해 파라미터 설정, 로봇 제어 스킬, 메인 시퀀스 노드를 **3단계 모듈로 분리(Modularization)**하였습니다.

---
폴더 및 파일 구조 (Module Architecture)

```text
Team_E2/ (bowl branch)
 ├── bowl_config.py        # 로봇 전역 파라미터, 속도, 강성 설정 및 실측 좌표 데이터
 ├── bowl_skills.py        # 그리퍼 개폐(DO 2/3번 핀) 및 바닥 반발력 감지 하강 서브루틴
 ├── scrub_bowl_main.py    # ROS 2 메인 노드 및 3대 핵심 세척 시퀀스 실행 스크립트
 └── README.md             # 프로젝트 브랜치 매뉴얼

---


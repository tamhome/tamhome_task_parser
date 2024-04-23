# Handy man用のタスク理解

## How to run

```bash
export OPENAI_API="api_key"
python3 script/task_parser_test.py
```

## How to launch

```bash
rosrun tamhome_task_parser task_parser_service.py
```

## How to call

```bash
> rosservice call /tamhome/task_parser/service "order: 'go to the living room'                   
extra_prompt: ''" 

---
next_skill: "move"
target: "living_room"
```

## 残開発タスク

- [ ] 部屋の情報をプロンプトに組み込む方法を確立

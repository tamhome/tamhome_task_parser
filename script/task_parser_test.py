#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import copy
import openai
from tamlib.utils import Logger


class TaskParser(Logger):
    def __init__(self) -> None:
        Logger.__init__(self)
        # 環境変数からAPIキーを取得
        self.api_key = os.environ.get("OPENAI_API")
        self.gpt_version = "gpt-4"

        if self.api_key is None:
            print("OPENAI_API is not set.")
            sys.exit()
        else:
            openai.api_key = self.api_key

        self.orig_handyman_prompt = [
            {"role": "system", "content": "あなたは家庭内で動作するホームサービスロボットで，オペレータの指示をもとに行動する必要があります．"},
            {"role": "system", "content": "指示文から，次に求められているローレベルの動作を推定してください．"},
            {"role": "system", "content": "あなたが実行可能な動作は，部屋や人の元へ移動する[move]，物体を見つける[find]，物体を把持する[grasp]，人に渡すための[deliver]，どこかにおく[put]です．"},
            {"role": "system", "content": "物体を把持する[grasp]の直前には必ず，物体を見つける[find]が必要です．"},
            {"role": "system", "content": "[find][deliver]の直前は必ず[move]が必要です．"},
            {"role": "system", "content": "指示文の内容を完遂し，次に行うべき動作がない場合は[done]と出力してください．"},
            {"role": "system", "content": "以下は例です．"},
            {"role": "user", "content": "Previous skills: [start], You are in [instruction_point], Instruction: Go to the living room, grasp the coke and bring it here."},
            {"role": "assistant", "content": "[move][living room]"},
            {"role": "user", "content": "Previous skills: [start][move], You are in [living room], Instruction: Go to the living room, grasp the coke and bring it here."},
            {"role": "assistant", "content": "[find][coke]"},
            {"role": "user", "content": "Previous skills: [start][move][find], You are in [living room], Instruction: Go to the living room, grasp the coke and bring it here."},
            {"role": "assistant", "content": "[grasp][coke]"},
            {"role": "user", "content": "Previous skills: [start][move][find][grasp], You are in [living room], Instruction: Go to the living room, grasp the coke and bring it here."},
            {"role": "assistant", "content": "[move][person]"},
            {"role": "user", "content": "Previous skills: [start][move][find][grasp][move], You are in [behind human], Instruction: Go to the living room, grasp the coke and bring it here."},
            {"role": "assistant", "content": "[deliver][coke]"},
            {"role": "user", "content": "Previous skills: [start][move][find][grasp][move][deliver], You are in [behind human], Instruction: Go to the living room, grasp the coke and bring it here."},
            {"role": "assistant", "content": "[done][none]"},
        ]

        self.handyman_prompt = copy.deepcopy(self.orig_handyman_prompt)
        self.previous_skills = "[start]"
        self.hsrb_location = "instruction_point"

    def extract_commands(self, s: str):
        """gptからの出力を整形する関数
        """
        # Split the string by '][' to separate the commands, then strip the brackets
        commands = s.strip("[]").split("][")
        return commands

    def action_parser(self, order_txt: str, previous_skills=None, hsrb_location=None) -> str:
        """HandyManのコマンド理解
        Args:
            order_txt (str): Go to the XXXX, grasp the YYYY and bring it here.
        Return:
            str: 次の動作と対象とする場所 or 物体
        """
        # self.logdebug(f"previous skills: {self.previous_skills}")
        # self.logdebug(f"herb location is: {self.hsrb_location}")

        openai.api_key = self.api_key
        self.handyman_prompt.append({"role": "user", "content": f"previous skills: {self.previous_skills}, You are in {self.hsrb_location}, Instruction: {order_txt}"})
        response = openai.ChatCompletion.create(
            model=self.gpt_version,
            messages=self.handyman_prompt
        )

        try:
            commands = self.extract_commands(response['choices'][0]['message']['content'])
            next_skill = commands[0]
            self.previous_skills += f"[{next_skill}]"
            if next_skill == "move":
                self.hsrb_location = commands[1]
        except Exception as e:
            self.logwarn(e)
            return "[done]"

        return commands

    def run(self):
        while True:
            text = input("instruction command >> ")
            trial = 0

            while trial < 10:
                trial += 1
                commands = cls.action_parser(text)
                try:
                    next_skill = commands[0]
                    target = commands[1]
                    self.loginfo(f"next skill is: {next_skill}")
                    self.loginfo(f"target is: {target}")
                    if next_skill == "done":
                        break
                except Exception as e:
                    self.logwarn(e)
                    self.loginfo(commands)
                    break


if __name__ == "__main__":
    cls = TaskParser()
    cls.run()

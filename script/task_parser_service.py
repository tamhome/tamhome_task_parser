#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import copy
import rospy
import openai
from tamlib.utils import Logger
from include.prompt_templates import PromptTemplates

from tamhome_task_parser.srv import ParseTask, ParseTaskResponse, ParseTaskRequest
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class TaskParserService(Logger, PromptTemplates):
    def __init__(self) -> None:
        Logger.__init__(self, loglevel="INFO")
        # 環境変数からAPIキーを取得
        self.api_key = os.environ.get("OPENAI_API")
        self.gpt_version = "gpt-4"

        if self.api_key is None:
            self.logerr("OPENAI_API is not set.")
            sys.exit()
        else:
            openai.api_key = self.api_key

        prompt_templetes = PromptTemplates()
        self.base_prompt = prompt_templetes.handyman_prompt

        self.orig_handyman_prompt = self.base_prompt
        self.loginfo(self.orig_handyman_prompt)
        self.reset_prompt()

        # ros interface
        rospy.Service("/tamhome/task_parser/service", ParseTask, self.run)
        rospy.Service("/tamhome/task_parser/reset_prompt", Empty, self.cb_reset_prompt)
        self.logsuccess("Task parser service is ready!")

    def extract_commands(self, s: str):
        """gptからの出力を整形する関数
        """
        # Split the string by '][' to separate the commands, then strip the brackets
        commands = s.strip("[]").split("][")
        return commands

    def delete(self):
        """デストラクタ
        """

    def reset_prompt(self):
        """プロンプトをベースのものにリセットし，現在位置，スキルもリセット
        """
        self.loginfo("initalize prompt and all settings")
        self.handyman_prompt = copy.deepcopy(self.orig_handyman_prompt)
        self.trial = 0
        self.previous_skills = "[start]"
        self.hsrb_location = "instruction_point"

    def cb_reset_prompt(self, req: EmptyRequest) -> EmptyResponse:
        self.reset_prompt()
        response = EmptyResponse()
        return response

    def action_parser(self, order_txt: str, previous_skills=None, hsrb_location=None) -> str:
        """HandyManのコマンド理解
        Args:
            order_txt (str): Go to the XXXX, grasp the YYYY and bring it here.
            previous_skills(str): [start][move][grasp]のようなこれまでに行ったスキルのリスト
            hsrb_location(str): HSRが今どこにいるのか
        Return:
            str: 次の動作と対象とする場所 と 対象とする[物体 or 部屋]
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

    def run(self, req: ParseTaskRequest) -> ParseTaskResponse:
        self.trial += 1
        order_txt = req.order
        extra_prompt = req.extra_prompt
        if extra_prompt is not None:
            self.loginfo(f"add prompt: {extra_prompt}")

        # コマンド理解
        commands = cls.action_parser(order_txt=order_txt)

        # レスポンスの作成
        try:
            next_skill = commands[0]
            target = commands[1]
            self.loginfo(f"next skill is: {next_skill}")
            self.loginfo(f"target is: {target}")
            if next_skill == "done":
                self.trial = 0
        except Exception as e:
            self.logwarn(e)
            self.loginfo(commands)

        response = ParseTaskResponse()
        response.next_skill = commands[0]
        response.target = commands[1]

        return response


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param("~loop_rate", 30)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = TaskParserService()

    rospy.on_shutdown(cls.delete)

    while not rospy.is_shutdown():
        try:
            pass
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()

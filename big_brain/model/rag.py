# 用于寻找最相似任务以作为prompt的参考
import json

import numpy as np

from sentence_transformers import SentenceTransformer, util
from config import RAG_MODEL, RAG_SIMILARITY_THRESHOLD

class RAGManager:
    def __init__(self, history_data):
        print("正在加载 Embedding 模型...")
        try:
            self.embedder = SentenceTransformer(RAG_MODEL)
            self.history_data = history_data
            self.corpus_embeddings = self._encode_corpus()
            print("RAG 模块初始化完成。")
        except Exception as e:
            print(f"[RAG] 模型加载失败，RAG 功能已禁用：{e}")
            self.embedder = None
            self.history_data = history_data
            self.corpus_embeddings = None

    def _encode_corpus(self):
        # 提取所有历史命令并计算向量
        commands = [item['command'] for item in self.history_data]
        if not commands:
            return None
        return self.embedder.encode(commands, convert_to_tensor=True)

    def retrieve(self, user_instruction, top_k=1, threshold=RAG_SIMILARITY_THRESHOLD):
        """
        检索最相似的历史任务，返回组装好的代码字符串
        """
        if self.embedder is None or self.corpus_embeddings is None:
            return ""

        query_embedding = self.embedder.encode(user_instruction, convert_to_tensor=True)
        scores = util.cos_sim(query_embedding, self.corpus_embeddings)[0]
        print("rag分数: ")
        for i,score in enumerate(scores):
            print(f"{i} : {score} {self.history_data[i]['command']}")
        # 找最高分
        best_idx = np.argmax(scores.cpu().numpy())
        best_score = scores[best_idx].item()

        if best_score >= threshold:
            # 获取对应的 record
            best_record = self.history_data[best_idx]
            
            # 组装返回字符串
            rag_text = f"# {best_record['command']}\n"
            rag_text += "\n".join(best_record['task_queue']) + "\n"
            return rag_text
        else:
            return "" # 相似度太低，不提供参考
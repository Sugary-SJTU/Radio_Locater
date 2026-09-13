flowchart LR
    A["输入：检测点 Mᵢ、示向度 θᵢ、误差 δ"]
    B["构造角域边界 θᵢ±δ"]
    C["统一为左侧可行半平面"]
    D["半平面交<br/>得到定位区域"]
    E["旋转卡壳<br/>求直径端点与 D"]
    F["构造直径圆<br/>并检验全部顶点"]
    G["输出：定位直径<br/>与覆盖判定"]

    A --> B --> C --> D --> E --> F --> G

    D --> H{"交集状态"}
    H -->|"空集"| H1["示向约束不相容"]
    H -->|"无界"| H2["增加检测点"]
    H -->|"退化"| H3["单点或线段处理"]

    classDef input fill:#DCEEFF,stroke:#3B82B6,stroke-width:1.7px,color:#163A5F,font-weight:bold;
    classDef geometry fill:#E8E4FF,stroke:#7C5CC4,stroke-width:1.7px,color:#3F2A70,font-weight:bold;
    classDef algorithm fill:#FFF0D8,stroke:#D97706,stroke-width:1.7px,color:#7A3D00,font-weight:bold;
    classDef output fill:#DCF5E5,stroke:#2F855A,stroke-width:1.7px,color:#14532D,font-weight:bold;
    classDef decision fill:#F3F4F6,stroke:#4B5563,stroke-width:1.7px,color:#111827,font-weight:bold;
    classDef warning fill:#FDE2E1,stroke:#C2413B,stroke-width:1.5px,color:#7F1D1D,font-weight:bold;

    class A input;
    class B,C,D geometry;
    class E,F algorithm;
    class G output;
    class H decision;
    class H1,H2,H3 warning;

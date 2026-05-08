\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{uestc-article}[2022/06/26 v1.0 电子科技大学学报类文件（屏蔽页眉版）]
\LoadClass[a4paper,11pt,twoside]{ctexart}

% 页面几何布局设置
\RequirePackage[hmargin=1.9cm,top=2.4cm,bottom=2.89cm,headheight=30pt,headsep=15pt]{geometry}
% 页眉页脚宏包（保留宏包加载，仅注释页眉配置）
\RequirePackage{fancyhdr}

% 字体设置
\RequirePackage{fontspec}
\setmainfont{times.ttf}[
  BoldFont = timesbd.ttf,
  ItalicFont = timesi.ttf,
  BoldItalicFont = timesbi.ttf
]
\setCJKmainfont{simsun.ttc}[AutoFakeBold]

% ==============================================
% 屏蔽页眉：将原页眉配置全部注释
% 如需恢复页眉，删除以下注释符即可
% ==============================================
% \pagestyle{fancy}
% \fancyhf{}
% \fancyhead[CO]{\zihao{-5}
%   \begin{tabular}{c}
%     电 \, 子 \, 科 \, 技 \, 大 \, 学 \, 学 \, 报\\[-1.2pt]
%     Journal of University of Electronic Science and Technology of China
%   \end{tabular}
% }
% \fancyhead[LO]{\zihao{-5}
%   \begin{tabular}{c}
%     第50卷 \, 第5期\\[-1.2pt]
%     2021年9月 
%   \end{tabular}
% }
% \fancyhead[RO]{\zihao{-5}
%   \begin{tabular}{c}
%     Vol.50 \, No.5\\[-1.2pt]
%     Sep. 2021
%   \end{tabular}
% }
% \fancyhead[LE]{\zihao{-5}\thepage}
% \fancyhead[CE]{\zihao{-5}电 \, 子 \, 科 \, 技 \, 大 \, 学 \, 学 \, 报}
% \fancyhead[RE]{\zihao{-5}第50卷}
% \renewcommand{\headrule}{
%   \ifodd\value{page}
%   \rule[2pt]{\textwidth}{0.3pt}\\[-10pt]\rule[2pt]{\textwidth}{0.3pt}
%   \else 
%   \rule[7pt]{\textwidth}{0.3pt}
%   \fi 
% }

% 作者（中文/英文）列表处理
\ExplSyntaxOn
\clist_clear_new:N \l_author_cn_clist 
\clist_clear_new:N \l_author_en_clist
\RenewDocumentCommand{\author}{s+m}
{
  \IfBooleanTF{#1}
  {
    \clist_put_right:Nn \l_author_cn_clist{#2\textsuperscript{*}}
  }
  {
    \clist_put_right:Nn \l_author_cn_clist{#2}
  }
}
\NewDocumentCommand{\authoren}{s+m}
{
  \IfBooleanTF{#1}
  {
    \clist_put_right:Nn \l_author_en_clist{#2\textsuperscript{*}}
  }
  {
    \clist_put_right:Nn \l_author_en_clist{#2}
  }
}

% 单位、英文标题定义
\def\address#1{\gdef\@address{#1}}
\def\titlen#1{\gdef\@titlen{#1}}
\def\addressen#1{\gdef\@addressen{#1}}

% 英文标题生成
\NewDocumentCommand{\maketitlen}{}
{
  \begin{center}
    {\zihao{3}\bfseries\@titlen}\par\vspace*{.5cm}
    {\zihao{5}\clist_use:Nnnn \l_author_en_clist{ ~and~ } { ,~ } { ,~and~ }}\par\vspace*{0.3cm}
    {\zihao{6}(\@addressen)}
  \end{center}
}

% 中文标题生成（重定义）
\RenewDocumentCommand{\maketitle}{}
{
  \begin{center}
    {\bfseries\heiti\zihao{-2}\@title}\par\vspace*{.5cm}
    {\zihao{4}\clist_use:Nn\l_author_cn_clist{，}}\par\vspace*{.3cm}
    {\zihao{6} （\@address）}
  \end{center}
}

% 中文摘要环境（重定义）
\RenewDocumentEnvironment{abstract}{+b}
{\par
  {\heiti【摘要】}\,\kaishu\zihao{-5} #1
}{\par}

% 中文关键词
\NewDocumentCommand{\keywords}{+m}
{
  {\zihao{-5}\heiti\makebox[5em][s]{关键词}}\hspace*{.5em} {\zihao{-5}\kaishu #1}
}

% 英文摘要环境
\NewDocumentEnvironment{abstracten}{+b}
{\par
  \textbf{Abstract}\hspace*{.5em}\zihao{5} #1
}{\par}

% 英文关键词
\NewDocumentCommand{\keywordsen}{+m}
{
  \textbf{Keywords}\hspace*{.5em}#1
}

% 附加信息（脚注）环境
\seq_new:N \l_info_seq 
\NewDocumentEnvironment{info}{+b}
{
  \group_begin: 
  \let\thefootnote\relax 
  \regex_split:nnN{\c{item}}{#1}\l_info_seq 
  \seq_map_inline:Nn \l_info_seq
  {
    \tl_if_empty:nF{##1}
    {
      \footnote{##1}
    }   
  }
}
{
  \group_end:
}

% 章节格式设置
\ctexset{
  section={
    format+=\zihao{4}\heiti\raggedright,
    beforeskip = 5pt,
    afterskip  = 3pt,
  },
  subsection={
    format+=\zihao{5}\heiti\raggedright,
    beforeskip = 5pt,
    afterskip  = 3pt,
  },
}

% 文档起始字号
\AtBeginDocument{\zihao{5}}

% 多列排版宏包
\RequirePackage{multicol}

% 参考文献标题重定义
\renewcommand{\refname}{\makebox[7em][s]{参考文献}}
\renewenvironment{thebibliography}[1]
{
  \ctexset{
    section={
      format=\zihao{5}\heiti\centering,
    }
  } 
  \section*{\refname}%
  \kaishu\zihao{-5}
  \@mkboth{\MakeUppercase\refname}{\MakeUppercase\refname}%
  \list{\@biblabel{\@arabic\c@enumiv}}%
       {\settowidth\labelwidth{\@biblabel{#1}}%
        \leftmargin\labelwidth
        \advance\leftmargin\labelsep
        \@openbib@code
        \usecounter{enumiv}%
        \let\p@enumiv\@empty
        \renewcommand\theenumiv{\@arabic\c@enumiv}}%
  \sloppy
  \clubpenalty4000
  \@clubpenalty \clubpenalty
  \widowpenalty4000%
  \sfcode`\.\@m}
{
  \def\@noitemerr
    {\@latex@warning{Empty `thebibliography' environment}}%
  \endlist}

% 中图分类号+文献标识码
\long\def\clc#1#2{\par{\zihao{-5}\heiti{中图分类号}}\hspace*{.5em}{ \kaishu #1} \hfil {\zihao{-5}\heiti{文献标识码}}\hspace*{.5em}{\kaishu #2 }}
\ExplSyntaxOff

% 结束输入
\endinput
---
layout: post
title:  "Desenvolvendo um teste de desempenho para o Kernel Samepage Merging (KSM)"
date:   2024-08-29 16:56:13 -0300
author: Pedro Demarchi Gomes
tags: kernel-linux
---

# Introdução

Esse relatório apresenta o projeto de contribuição para o Kernel Linux desenvolvido durante a disciplina MO806A.

O projeto consiste em desenvolver código para testar a performance da feature de economia de memória Kernel Samepage Merging (KSM) ao se utilizar huge pages. Esses testes são chamados de Kselftests, e estão contidos na árvore do Kernel.

A seção kselftests apresenta o framework de testes Kselftest. As seções ksm e thp apresentam a feature KSM e o sistema de Transparent Huge Pages, respectivamente, para que na seção thpvsksm seja apresentado o problema que ocorre ao se habilitar os dois sistemas juntos. A contribuição, que está relacionada a esse problema, é apresentada na seção contribuicao.


# Kselftests

Os Kselftests são um conjunto de testes unitários e testes de regressão para o Kernel Linux{% cite KselftestsWiki %} , que podem ser encontrados no diretório tools/testing/selftest na árvore do Kernel. Esses testes são executados diariamente em diversas árvores do Kernel.

Cada teste tem como objetivo executar caminhos específicos de código a fim de encontrar bugs e regressões.

Os testes são organizados em grupos, onde cada um testa uma parte do Kernel diferente (memória, scheduler, KVM, etc). O usuário pode executar todos os testes, apenas um grupo de testes, ou apenas um teste em específico.

Um dos requisitos dos Kselftests é que o tempo de execução de todos os testes não ultrapasse 20 minutos.

A árvore de desenvolvimento dos Kselftests é mantida pela desenvolvedora Shuah Khan, e pode ser encontrada em {% cite KselftestsTree %}

# Kernel Samepage Merging (KSM)

Kernel Samepage Merging é uma feature de economia de memória do Kernel Linux. Quando ativada, o Kernel passa a varrer a memória em busca de páginas de memória iguais. Ao encontrar páginas duplicadas, uma delas é mantida e compartilhada entre os processos, as demais são liberadas, economizando o uso de memória. A página compartilhada pelos processos é marcada como copy on write (COW){% cite KsmLwn %}.

O KSM foi desenvolvido para ser utilizado em hosts que executam diversas máquinas virtuais. Nesses sistemas, uma parte significativa da memória utilizada pelos guests é idêntica, logo a ativação do KSM gera uma grande economia de memória.

A figura abaixo mostra uma representação de uma máquina física, utilizando o KSM, executando três máquinas virtuais. Parte das páginas utilizadas pelas máquinas virtuais são compartilhadas pelo KSM, e as demais são privadas de cada máquina virtual.

![](/assets/ksm_vm.png)
*Máquina host com KSM ativado compartilhando páginas de máquinas virtuais {% cite anatomyKSM %}*

# Transparent Huge Pages

Transparent Huge Pages (THP) é um sistema implementado no Kernel Linux para utilizar huge pages de maneira transparente ao programador. Com esse sistema as páginas base, normalmente de tamanho 4K, são escaneadas e promovidas quando pertinente para huge pages, normalmente de tamanho 2MB. As huge pages também podem ser "quebradas" em páginas base quando for necessário.

O uso de huge pages diminui o número de páginas utilizadas pelas aplicações, o que diminui o número de TLB misses e aumenta o desempenho. {% cite hornyack %} mostra que algumas aplicações podem passar 58% de seu tempo de execução tratando TLB misses. Logo, o uso de huge pages é essencial para aumentar o desempenho de diversas aplicações. 

# THP vs KSM

Utilizar THP com KSM pode levar a uma grande queda de desempenho do sistema, devido ao KSM “quebrar” huge pages ao encontrar páginas base duplicadas dentro de uma huge page. Isso leva a um baixo uso de huge pages que pode anular seus benefícios, deixando para o sistema apenas o overhead do THP.


Devido a esse problema diversas modificações, tanto no KSM quando no THP, estão sendo propostas. Pensando nisso, o projeto apresentado nesse relatório consiste em contribuir com testes para medir a velocidade do sistema de KSM utilizando THP.

# Contribuição

A contribuição realizada nesse projeto se deu no código do Kselftest que testa o KSM. Nele foi adicionado um parâmetro para testar a velocidade de compartilhamento de huge pages.

Esse teste tem como objetivo auxiliar os desenvolvedores do sistema de KSM, avaliando a performance do sistema ao interagir com huge pages.

As contribuições no Kernel Linux são feitas através de patches, que são enviados para listas de email. Existem diversas listas de email, e cada uma recebe patches de partes do Kernel diferentes.

Os patches são gerenciados pelos mantenedores, que são responsáveis por aprová-los, reprová-los, ou sugerir melhorias. Cada mantenedor possui uma árvore do Kernel onde, caso o patch seja aprovado, ele é aplicado.

Existem diversos mantenedores, responsáveis por gerenciar o desenvolvimento de diversas partes do Kernel.

Para saber para qual lista de email enviar um patch basta executar o script get_maintainer.pl, que se encontra na árvore do Kernel, passando como parâmetro o caminho para seu patch. Ele retornará o nome e email dos mantenedores e as listas de email para os quais o patch deve ser enviado.

As próximas subseções descrevem detalhes sobre o código e a interação com a comunidade ao contribuir. 

## Código

O teste para medir a velocidade de compartilhamento de páginas do KSM é simples. Ele se encontra em tools/testing/selftest/vm/ksm\_tests.c, e recebe como parâmetros o tipo de teste e o tamanho da região de memória duplicada. Nele é medido o tempo de execução da função ksm\_merge\_pages, mostrada abaixo.

{% highlight c %}
static int ksm_merge_pages(void *addr, size_t size, 
    struct timespec start_time, int timeout)                                                                                              
{
  if (madvise(addr, size, MADV_MERGEABLE)) {
    perror("madvise");
    return 1;
  }
  if (ksm_write_sysfs(KSM_FP("run"), 1))
    return 1;

  /* Since merging occurs only after 2 scans, 
  make sure to get at least 2 full scans */
  if (ksm_do_scan(2, start_time, timeout))
    return 1;

  return 0;
}
{% endhighlight %}

Essa função primeiramente habilita a região de memória utilizada no teste a ser escaneada pelo KSM. Em seguida, ela executa o KSM. Por último, ela comanda o KSM a escanear duas vezes a memória, pois apenas na segunda ocorrem os compartilhamentos de página.

Para adicionar a opção de testar a velocidade de compartilhamento de huge pages, basta alocar a região de memória a ser compartilhada utilizando huge pages.

Foram desenvolvidas duas versões do patch, a primeira e a segunda versão podem ser encontradas nos apêndices A e B, respectivamente.

Na primeira versão é utilizada a chamada de sistema madvise para alocar a região de memória a ser testada com huge pages, porém, essa forma faz com que não haja controle da quantidade de huge pages que foram alocadas para aquela região, o que pode interferir nos testes.

Na segunda versão é utilizada a função allocate_transhuge para alocar as huge pages. Essa função tenta alocar huge pages e retorna -1 caso não tenha conseguido. Dessa forma é possível saber quantas huge pages foram alocadas na região. O número de páginas normais e huge pages alocadas é informado ao usuário.

A memória é alocada por partes de tamanho HPAGE_SIZE, que corresponde ao tamanho de uma Huge Page, como mostrado no trecho de código abaixo. Para cada parte é chamada a função allocate\_transhuge, e de acordo com seu retorno são contados o número de páginas alocadas de cada tipo.

{% highlight c %}
n_normal_pages = 0;
n_huge_pages = 0;
for (void *p = map_ptr; p < map_ptr + len; 
    p += HPAGE_SIZE) {
  if (allocate_transhuge(p, pagemap_fd) < 0)
    n_normal_pages++;
  else
    n_huge_pages++;
}
{% endhighlight %}

A função allocate\_transhuge, mostrada abaixo, utiliza a interface do Kernel Linux chamada pagemap{% cite pagemap %}, que permite o usuário acessar a tabela de páginas de um programa, para verificar o tipo de página alocada. Ela pode ser acessada em /proc/$<$pid$>$/pagemap, sendo $<$pid$>$ o número de identificação do processo ou self, caso o processo queira acessar sua própria tabela de páginas.

{% highlight c %}
int64_t allocate_transhuge(void *ptr, int pagemap_fd)
{
  uint64_t ent[2];

  /* drop pmd */
  if (mmap(ptr, HPAGE_SIZE, PROT_READ | PROT_WRITE,
    MAP_FIXED | MAP_ANONYMOUS |
    MAP_NORESERVE | MAP_PRIVATE, -1, 0) != ptr)
    errx(2, "mmap transhuge");

  if (madvise(ptr, HPAGE_SIZE, MADV_HUGEPAGE))
    err(2, "MADV_HUGEPAGE");

  /* allocate transparent huge page */
  *(volatile void **)ptr = ptr;

  if (pread(pagemap_fd, ent, sizeof(ent),
   (uintptr_t)ptr >> (PAGE_SHIFT - 3)) != sizeof(ent))
    err(2, "read pagemap");

  if (PAGEMAP_PRESENT(ent[0]) && 
    PAGEMAP_PRESENT(ent[1]) &&
    PAGEMAP_PFN(ent[0]) + 1 == PAGEMAP_PFN(ent[1]) &&
    !(PAGEMAP_PFN(ent[0]) & 
    ((1 << (HPAGE_SHIFT - PAGE_SHIFT)) - 1)))
    return PAGEMAP_PFN(ent[0]); 
   
  return -1;
} 
{% endhighlight %}

Primeiramente, as informações da página física mapeada pelo endereço virtual ptr e sua sucessora são armazenados no vetor ent. Para que tenha sido alocada uma huge page, as duas páginas devem estar presentes, ou seja, não estarem em swap, o Page Frame Number (PFN) da segunda deve ser o sucessor da primeira, e o PFN da primeira deve ser múltiplo de 512, pois uma huge page possui 512 páginas normais.

A organização das páginas físicas de memória mapeadas por huge pages é detalhada em {% cite hugepageLwn %}.

Para executar esse teste basta compilar os Kselftests e executar sudo ./ksm\_tests -H -s $<$N$>$, onde $N$ é o tamanho em MB da região duplicada, no diretório tools/testing/selftests/vm.

Um exemple de saída do teste com uma região duplicada de 100MB é apresentado abaixo.
{% highlight c %}
Number of normal pages:    0
Number of huge pages:    50
Total size:    100 MiB
Total time:    0.178690991 s
Average speed:  559.625 MiB/s
{% endhighlight %}

O código completo do teste pode ser encontrado em {% cite kernelKSM %}.

## Comunidade

Os patches foram enviados para os emails obtidos através do script get_maintainer.pl.

A primeira versão do patch enviado não teve respostas, e dias depois foi enviada a segunda versão.

A segunda versão foi primeiramente aplicada na árvore linux-mm do Kernel, que recebe patches relacionados ao sistema de memória. Depois, foi selecionada pelo seu mantenedor Andrew Morton para ser enviada para o Mainline Kernel, e foi aplicada no Kernel v5.16-rc3. O commit em que o patch foi aplicado pode ser encontrado em {% cite Patch %}.

Após ser aplicado no Mainline Kernel, o patch também foi aplicado na árvore de desenvolvimento dos Kselftests.

# Conclusão

Esse projeto possibilitou o contato com a comunidade de desenvolvedores do Kernel Linux, o aprendizado das ferramentas utilizadas para fazer uma contribuição e para testar alterações no Kernel, e principalmente o aprendizado do funcionamento do sistema de gerenciamento de memória.

Além da contribuição em si, a leitura de listas de email e portais de informação sobre o Kernel Linux, como o LWN.net{% cite LWN %}, durante todo o desenvolvimento do projeto, ajudou a acompanhar o processo de discussão que ocorre até uma grande alteração chegar a ser aplicada no Mainline Kernel.

{% bibliography --cited %}

# Apêndice A

Nesse apêndice é apresentada a primeira versão do patch desenvolvido no projeto.

```diff
Signed-off-by: Pedro Demarchi Gomes <pedrodemargomes@gmail.com>
---
 tools/testing/selftests/vm/ksm_tests.c | 40 +++++++++++++++++++-------
 1 file changed, 30 insertions(+), 10 deletions(-)

diff --git a/tools/testing/selftests/vm/ksm_tests.c b/tools/testing/selftests/vm/ksm_tests.c
index b61dcdb44c5b..92b716565d9c 100644
--- a/tools/testing/selftests/vm/ksm_tests.c
+++ b/tools/testing/selftests/vm/ksm_tests.c
@@ -5,6 +5,7 @@
 #include <time.h>
 #include <string.h>
 #include <numa.h>
+#include <err.h>

 #include "../kselftest.h"
 #include "../../../../include/vdso/time64.h"
@@ -34,7 +35,8 @@ enum ksm_test_name {
        CHECK_KSM_ZERO_PAGE_MERGE,
        CHECK_KSM_NUMA_MERGE,
        KSM_MERGE_TIME,
-       KSM_COW_TIME
+       KSM_COW_TIME,
+       KSM_MERGE_TIME_HUGE_PAGES
 };

 static int ksm_write_sysfs(const char *file_path, unsigned long val)
@@ -99,6 +101,9 @@ static void print_help(void)
               " -U (page unmerging)\n"
               " -P evaluate merging time and speed.\n"
               "    For this test, the size of duplicated memory area (in MiB)\n"
+              "    must be provided using -s option\n"
+                  " -H evaluate merging time and speed of huge pages.\n"
+              "    For this test, the size of duplicated memory area (in MiB)\n"
               "    must be provided using -s option\n"
               " -C evaluate the time required to break COW of merged pages.\n\n");

@@ -118,10 +123,14 @@ static void print_help(void)
        exit(0);
 }

-static void  *allocate_memory(void *ptr, int prot, int mapping, char data, size_t map_size)
+static void  *allocate_memory(void *ptr, int prot, int mapping, char data, size_t map_size,
+                                bool huge_page)
 {
        void *map_ptr = mmap(ptr, map_size, PROT_WRITE, mapping, -1, 0);

+       if (huge_page && madvise(map_ptr, map_size, MADV_HUGEPAGE))
+               err(2, "MADV_HUGEPAGE");
+
        if (!map_ptr) {
                perror("mmap");
                return NULL;
@@ -250,7 +259,7 @@ static int check_ksm_merge(int mapping, int prot, long page_count, int timeout,
        }

        /* fill pages with the same data and merge them */
-       map_ptr = allocate_memory(NULL, prot, mapping, '*', page_size * page_count);
+       map_ptr = allocate_memory(NULL, prot, mapping, '*', page_size * page_count, false);
        if (!map_ptr)
                return KSFT_FAIL;

@@ -282,7 +291,7 @@ static int check_ksm_unmerge(int mapping, int prot, int timeout, size_t page_siz
        }

        /* fill pages with the same data and merge them */
-       map_ptr = allocate_memory(NULL, prot, mapping, '*', page_size * page_count);
+       map_ptr = allocate_memory(NULL, prot, mapping, '*', page_size * page_count, false);
        if (!map_ptr)
                return KSFT_FAIL;

@@ -325,7 +334,7 @@ static int check_ksm_zero_page_merge(int mapping, int prot, long page_count, int
                return KSFT_FAIL;

        /* fill pages with zero and try to merge them */
-       map_ptr = allocate_memory(NULL, prot, mapping, 0, page_size * page_count);
+       map_ptr = allocate_memory(NULL, prot, mapping, 0, page_size * page_count, false);
        if (!map_ptr)
                return KSFT_FAIL;

@@ -416,7 +425,7 @@ static int check_ksm_numa_merge(int mapping, int prot, int timeout, bool merge_a
        return KSFT_FAIL;
 }

-static int ksm_merge_time(int mapping, int prot, int timeout, size_t map_size)
+static int ksm_merge_time(int mapping, int prot, int timeout, size_t map_size, bool huge_page)
 {
        void *map_ptr;
        struct timespec start_time, end_time;
@@ -424,7 +433,7 @@ static int ksm_merge_time(int mapping, int prot, int timeout, size_t map_size)

        map_size *= MB;

-       map_ptr = allocate_memory(NULL, prot, mapping, '*', map_size);
+       map_ptr = allocate_memory(NULL, prot, mapping, '*', map_size, huge_page);
        if (!map_ptr)
                return KSFT_FAIL;

@@ -466,7 +475,7 @@ static int ksm_cow_time(int mapping, int prot, int timeout, size_t page_size)
        /* page_count must be less than 2*page_size */
        size_t page_count = 4000;

-       map_ptr = allocate_memory(NULL, prot, mapping, '*', page_size * page_count);
+       map_ptr = allocate_memory(NULL, prot, mapping, '*', page_size * page_count, false);
        if (!map_ptr)
                return KSFT_FAIL;

@@ -541,7 +550,7 @@ int main(int argc, char *argv[])
        bool merge_across_nodes = KSM_MERGE_ACROSS_NODES_DEFAULT;
        long size_MB = 0;

-       while ((opt = getopt(argc, argv, "ha:p:l:z:m:s:MUZNPC")) != -1) {
+       while ((opt = getopt(argc, argv, "ha:p:l:z:m:s:MUZNPCH")) != -1) {
                switch (opt) {
                case 'a':
                        prot = str_to_prot(optarg);
@@ -598,6 +607,9 @@ int main(int argc, char *argv[])
                case 'C':
                        test_name = KSM_COW_TIME;
                        break;
+               case 'H':
+                       test_name = KSM_MERGE_TIME_HUGE_PAGES;
+                       break;
                default:
                        return KSFT_FAIL;
                }
@@ -645,12 +657,20 @@ int main(int argc, char *argv[])
                        return KSFT_FAIL;
                }
                ret = ksm_merge_time(MAP_PRIVATE | MAP_ANONYMOUS, prot, ksm_scan_limit_sec,
-                                    size_MB);
+                                    size_MB, false);
                break;
        case KSM_COW_TIME:
                ret = ksm_cow_time(MAP_PRIVATE | MAP_ANONYMOUS, prot, ksm_scan_limit_sec,
                                   page_size);
                break;
+       case KSM_MERGE_TIME_HUGE_PAGES:
+               if (size_MB == 0) {
+                       printf("Option '-s' is required.\n");
+                       return KSFT_FAIL;
+               }
+               ret = ksm_merge_time(MAP_PRIVATE | MAP_ANONYMOUS, prot, ksm_scan_limit_sec,
+                                    size_MB, true);
+               break;
        }

        if (ksm_restore(&ksm_sysfs_old)) {
-- 
2.25.1

```


# Apêndice B

Nesse apêndice é apresentada a segunda versão do patch desenvolvido no projeto.

```diff

Add test case of KSM merging time using mostly huge pages

Signed-off-by: Pedro Demarchi Gomes <pedrodemargomes@gmail.com>
---
 tools/testing/selftests/vm/ksm_tests.c | 125 ++++++++++++++++++++++++-
 1 file changed, 124 insertions(+), 1 deletion(-)

diff --git a/tools/testing/selftests/vm/ksm_tests.c b/tools/testing/selftests/vm/ksm_tests.c
index b61dcdb44c5b..61f3360a19df 100644
--- a/tools/testing/selftests/vm/ksm_tests.c
+++ b/tools/testing/selftests/vm/ksm_tests.c
@@ -5,6 +5,10 @@
 #include <time.h>
 #include <string.h>
 #include <numa.h>
+#include <unistd.h>
+#include <fcntl.h>
+#include <stdint.h>
+#include <err.h>
 
 #include "../kselftest.h"
 #include "../../../../include/vdso/time64.h"
@@ -18,6 +22,15 @@
 #define KSM_MERGE_ACROSS_NODES_DEFAULT true
 #define MB (1ul << 20)
 
+#define PAGE_SHIFT 12
+#define HPAGE_SHIFT 21
+
+#define PAGE_SIZE (1 << PAGE_SHIFT)
+#define HPAGE_SIZE (1 << HPAGE_SHIFT)
+
+#define PAGEMAP_PRESENT(ent)	(((ent) & (1ull << 63)) != 0)
+#define PAGEMAP_PFN(ent)	((ent) & ((1ull << 55) - 1))
+
 struct ksm_sysfs {
 	unsigned long max_page_sharing;
 	unsigned long merge_across_nodes;
@@ -34,6 +47,7 @@ enum ksm_test_name {
 	CHECK_KSM_ZERO_PAGE_MERGE,
 	CHECK_KSM_NUMA_MERGE,
 	KSM_MERGE_TIME,
+	KSM_MERGE_TIME_HUGE_PAGES,
 	KSM_COW_TIME
 };
 
@@ -99,6 +113,9 @@ static void print_help(void)
 	       " -U (page unmerging)\n"
 	       " -P evaluate merging time and speed.\n"
 	       "    For this test, the size of duplicated memory area (in MiB)\n"
+	       "    must be provided using -s option\n"
+				 " -H evaluate merging time and speed of area allocated mostly with huge pages\n"
+	       "    For this test, the size of duplicated memory area (in MiB)\n"
 	       "    must be provided using -s option\n"
 	       " -C evaluate the time required to break COW of merged pages.\n\n");
 
@@ -416,6 +433,101 @@ static int check_ksm_numa_merge(int mapping, int prot, int timeout, bool merge_a
 	return KSFT_FAIL;
 }
 
+int64_t allocate_transhuge(void *ptr, int pagemap_fd)
+{
+	uint64_t ent[2];
+
+	/* drop pmd */
+	if (mmap(ptr, HPAGE_SIZE, PROT_READ | PROT_WRITE,
+				MAP_FIXED | MAP_ANONYMOUS |
+				MAP_NORESERVE | MAP_PRIVATE, -1, 0) != ptr)
+		errx(2, "mmap transhuge");
+
+	if (madvise(ptr, HPAGE_SIZE, MADV_HUGEPAGE))
+		err(2, "MADV_HUGEPAGE");
+
+	/* allocate transparent huge page */
+	*(volatile void **)ptr = ptr;
+
+	if (pread(pagemap_fd, ent, sizeof(ent),
+			(uintptr_t)ptr >> (PAGE_SHIFT - 3)) != sizeof(ent))
+		err(2, "read pagemap");
+
+	if (PAGEMAP_PRESENT(ent[0]) && PAGEMAP_PRESENT(ent[1]) &&
+	    PAGEMAP_PFN(ent[0]) + 1 == PAGEMAP_PFN(ent[1]) &&
+	    !(PAGEMAP_PFN(ent[0]) & ((1 << (HPAGE_SHIFT - PAGE_SHIFT)) - 1)))
+		return PAGEMAP_PFN(ent[0]);
+
+	return -1;
+}
+
+static int ksm_merge_hugepages_time(int mapping, int prot, int timeout, size_t map_size)
+{
+	void *map_ptr, *map_ptr_orig;
+	struct timespec start_time, end_time;
+	unsigned long scan_time_ns;
+	int pagemap_fd, n_normal_pages, n_huge_pages;
+
+	map_size *= MB;
+	size_t len = map_size;
+
+	len -= len % HPAGE_SIZE;
+	map_ptr_orig = mmap(NULL, len + HPAGE_SIZE, PROT_READ | PROT_WRITE,
+			MAP_ANONYMOUS | MAP_NORESERVE | MAP_PRIVATE, -1, 0);
+	map_ptr = map_ptr_orig + HPAGE_SIZE - (uintptr_t)map_ptr_orig % HPAGE_SIZE;
+
+	if (map_ptr_orig == MAP_FAILED)
+		err(2, "initial mmap");
+
+	if (madvise(map_ptr, len + HPAGE_SIZE, MADV_HUGEPAGE))
+		err(2, "MADV_HUGEPAGE");
+
+	pagemap_fd = open("/proc/self/pagemap", O_RDONLY);
+	if (pagemap_fd < 0)
+		err(2, "open pagemap");
+
+	n_normal_pages = 0;
+	n_huge_pages = 0;
+	for (void *p = map_ptr; p < map_ptr + len; p += HPAGE_SIZE) {
+		if (allocate_transhuge(p, pagemap_fd) < 0)
+			n_normal_pages++;
+		else
+			n_huge_pages++;
+	}
+	printf("Number of normal pages:    %d\n", n_normal_pages);
+	printf("Number of huge pages:    %d\n", n_huge_pages);
+
+	memset(map_ptr, '*', len);
+
+	if (clock_gettime(CLOCK_MONOTONIC_RAW, &start_time)) {
+		perror("clock_gettime");
+		goto err_out;
+	}
+	if (ksm_merge_pages(map_ptr, map_size, start_time, timeout))
+		goto err_out;
+	if (clock_gettime(CLOCK_MONOTONIC_RAW, &end_time)) {
+		perror("clock_gettime");
+		goto err_out;
+	}
+
+	scan_time_ns = (end_time.tv_sec - start_time.tv_sec) * NSEC_PER_SEC +
+		       (end_time.tv_nsec - start_time.tv_nsec);
+
+	printf("Total size:    %lu MiB\n", map_size / MB);
+	printf("Total time:    %ld.%09ld s\n", scan_time_ns / NSEC_PER_SEC,
+	       scan_time_ns % NSEC_PER_SEC);
+	printf("Average speed:  %.3f MiB/s\n", (map_size / MB) /
+					       ((double)scan_time_ns / NSEC_PER_SEC));
+
+	munmap(map_ptr_orig, len + HPAGE_SIZE);
+	return KSFT_PASS;
+
+err_out:
+	printf("Not OK\n");
+	munmap(map_ptr_orig, len + HPAGE_SIZE);
+	return KSFT_FAIL;
+}
+
 static int ksm_merge_time(int mapping, int prot, int timeout, size_t map_size)
 {
 	void *map_ptr;
@@ -541,7 +653,7 @@ int main(int argc, char *argv[])
 	bool merge_across_nodes = KSM_MERGE_ACROSS_NODES_DEFAULT;
 	long size_MB = 0;
 
-	while ((opt = getopt(argc, argv, "ha:p:l:z:m:s:MUZNPC")) != -1) {
+	while ((opt = getopt(argc, argv, "ha:p:l:z:m:s:MUZNPCH")) != -1) {
 		switch (opt) {
 		case 'a':
 			prot = str_to_prot(optarg);
@@ -595,6 +707,9 @@ int main(int argc, char *argv[])
 		case 'P':
 			test_name = KSM_MERGE_TIME;
 			break;
+		case 'H':
+			test_name = KSM_MERGE_TIME_HUGE_PAGES;
+			break;
 		case 'C':
 			test_name = KSM_COW_TIME;
 			break;
@@ -647,6 +762,14 @@ int main(int argc, char *argv[])
 		ret = ksm_merge_time(MAP_PRIVATE | MAP_ANONYMOUS, prot, ksm_scan_limit_sec,
 				     size_MB);
 		break;
+	case KSM_MERGE_TIME_HUGE_PAGES:
+		if (size_MB == 0) {
+			printf("Option '-s' is required.\n");
+			return KSFT_FAIL;
+		}
+		ret = ksm_merge_hugepages_time(MAP_PRIVATE | MAP_ANONYMOUS, prot,
+				ksm_scan_limit_sec, size_MB);
+		break;
 	case KSM_COW_TIME:
 		ret = ksm_cow_time(MAP_PRIVATE | MAP_ANONYMOUS, prot, ksm_scan_limit_sec,
 				   page_size);
-- 
2.25.1
```



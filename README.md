# PCBS-IdealObserver

Programmation d’un réseau récurrent de neurones capable d’implémenter un estimateur de type ‘observateur idéal’.


<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table des matières**

- [Programmation d’un réseau récurrent de neurones capable d’implémenter un estimateur de type ‘observateur idéal’.](#programmation-d-un-reseau-recurrent)
    - [Introduction](#introduction)
    - [Programmation du réseau](#programmation-du-reseau)
        - [Activité en entrée du réseau](#activite-en-entree-du-reseau)
        - [Dynamique du réseau](#dynamique)
        - [Poids de filtrage](#poids-de-filtrage)
        - [Activité en sortie du réseau](#activite-en-sortie-du-reseau)
        - [Estimations de l’orientation et de la fréquence spatiale encodées par le réseau ](#estimations)
        - [Moyenne et variance des estimations](#moyenne-et-variance-des-estimations)
        - [Essai avec un réseau de 20 × 20 neurones](#essai-avec-un-reseau-20x20-neurones)
     
    - [Conclusion](#conclusion)
    - [Références](#references)
    - [Remarques finales](#remarques)

<!-- markdown-toc end -->

## Introduction <a name="introduction"></a>


De nombreux neurones du cortex visuel primaire sont sensibles à l’orientation du stimulus présenté. La courbe représentant l’activité de réponse d’un neurone en fonction de l’orientation présentée, appelée courbe d’accord (<i>tuning curve</i>), est typiquement en forme de cloche, la position du maximum correspondant à l’orientation préférentielle du neurone. Elle permet de prédire la réponse moyenne d’un neurone à une orientation donnée. Cependant, la réponse individuelle pour une même orientation varie d’un essai à l’autre en raison du bruit neuronal. Cette variabilité est particulièrement visible lorsque l’on trace l’activité d’une population entière de neurones en réponse à une grille d’une orientation θ donnée. Pour un essai donné, en traçant l’activité de chaque neurone de la population en fonction de son orientation préférentielle, on obtient un pic très bruité. Un autre essai avec la même orientation du stimulus conduirait à une courbe similaire, mais avec une réponse légèrement différente pour chaque neurone, et donc une position du pic central différente.


Il s’agit alors pour le cerveau d’estimer la valeur de l’orientation encodée dans cette activité bruitée d’une population de neurones. Une des méthodes classiquement proposées pour effectuer cette tâche s’appuie sur le vecteur population de neurones. Il s’agit d’un estimateur facile à implémenter et sans biais, c’est-à-dire que l’estimation ainsi obtenue (qui varie d’un essai à l’autre en même temps que l’activité de la population varie), est égale en moyenne (sur les essais) à l’orientation présentée. Toutefois, cet estimateur n’est pas optimal car sa variance sur l’estimation est bien supérieure à la borne inférieure sur la variance pour un estimateur sans biais (borne de Cramér-Rao). Or idéalement, l’estimateur devrait avoir une variance minimale, c’est-à-dire que l’estimation devrait varier le moins possible d’un essai à l’autre, lorsque l’orientation est fixée. Cette borne inférieure est dictée par la structure du bruit neuronal ; elle est atteinte pour l’estimateur dit du maximum de vraisemblance : on parle d’observateur idéal, car il optimise l’estimation compte tenu du bruit.
 L’estimation de variables encodées dans l’activité de populations de neurones ne se limite pas toutefois à l’orientation, et d’autres variables sensorielles ou motricielles peuvent être encodées. Dans ce projet nous considèrerons deux variables : l’orientation et la fréquence spatiale d’une grille. 

L’objectif de ce projet est de simuler un réseau de neurones récurrents capable d’implémenter un estimateur sans biais de ces deux variables, et dont la variance est égale à (ou très proche de, suivant le type de bruit) la variance minimale atteinte par l’estimateur au maximum de vraisemblance. On cherche donc à implémenter un estimateur de type observateur idéal. Pour cela, le principe consiste à utiliser un réseau de neurones récurrents dont la fonction d’activation comprend une normalisation divisive (<i>divisive normalization</i>), dont l’expression sera explicitée plus loin, reproduisant les fonctions d’activations observées pour des neurones du cortex visuel primaire. Il a en effet été prouvé (Denève et al., 1999) que le réseau de neurones qui en résulte permet d’implémenter un estimateur dont la variance est égale à la variance minimale atteinte par le maximum de vraisemblance dans le cas d’un bruit neuronal indépendant du taux de décharge, et très proche de la valeur minimale dans le cas plus réaliste, considéré ici, d’un bruit poissonnien. 

## Programmation du réseau <a name="programmation-du-reseau"></a>

Le réseau que nous considérons modélise une colonne corticale constituée d’une seule couche de neurones (unités ij) dont les champs récepteurs sont identiques, mais qui diffèrent par leurs orientations et fréquences spatiales préférentielles. Chaque neurone est repéré par deux indices, et le neurone ij est caractérisé par son orientation préférentielle <a href="https://www.codecogs.com/eqnedit.php?latex=\theta_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta_{i}" title="\theta_{i}" /></a>	et sa fréquence spatiales <a href="https://www.codecogs.com/eqnedit.php?latex=\lambda_{j}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\lambda_{j}" title="\lambda_{j}" /></a>. Le réseau reçoit une activité en entrée provenant d’une couche précédente, qui représente soit une autre couche corticale soit le noyau géniculé latéral. (Nous ne modélisons donc pas explicitement cette couche, et nous concentrons sur l’entrée qu’elle fournit à chaque 
neurone de notre réseau).  On désignera l’activité du réseau qui en résulte sous le terme d’activité en sortie (<i>output activity</i>).

### Activité en entrée du réseau <a name="#activite-en-entree-du-reseau"></a>

L’entrée fournie au réseau dépend de l’orientation θ et de la fréquence spatiale λ du stimulus présenté, qui sont encodées dans la couche précédente. Pour des valeurs de θ et λ données, l’activité totale en entrée du neurone ij,                                       <a href="https://www.codecogs.com/eqnedit.php?latex=a_{ij}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?a_{ij}" title="a_{ij}" /></a>, est la somme de deux termes : le premier correspond à l’entrée moyenne  <a href="https://www.codecogs.com/eqnedit.php?latex=f_{ij}(\theta,\lambda)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?f_{ij}(\theta,\lambda)" title="f_{ij}(\theta,\lambda)" /></a>, le second <a href="https://www.codecogs.com/eqnedit.php?latex=\xi_{ij}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\xi_{ij}" title="\xi_{ij}" /></a> à un terme de bruit autour de cette activité moyenne en entrée :

<a href="https://www.codecogs.com/eqnedit.php?latex=a_{ij}=&space;f_{ij}&space;(\theta&space;,\lambda&space;)&plus;&space;\xi_{ij}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?a_{ij}=&space;f_{ij}&space;(\theta&space;,\lambda&space;)&plus;&space;\xi_{ij}" title="a_{ij}= f_{ij} (\theta ,\lambda )+ \xi_{ij}" /></a>

L’activité moyenne en entrée du neurone ij pour un stimulus d’orientation θ, de longueur d’onde spatiale λ et de contraste C, <a href="https://www.codecogs.com/eqnedit.php?latex=f_{ij}(\theta,\lambda)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?f_{ij}(\theta,\lambda)" title="f_{ij}(\theta,\lambda)" /></a>, est choisie de sorte à reproduire des courbes d’accord (<i>tuning curves</i>) physiologiquement réalistes – des profils en forme de cloche dont l’amplitude est proportionnelle au contraste. Pour cela, on choisit de prendre des fonctions circulaires normales avec un terme d’activité spontanée ν en plus :

<a href="https://www.codecogs.com/eqnedit.php?latex=$$f_{ij}&space;(\theta,\lambda)&space;=&space;KCexp&space;\left&space;(&space;\frac{&space;cos\left&space;(\theta-\theta_i&space;\right)-&space;1}{\sigma&space;_{\theta&space;}^{2}}\:&space;&plus;&space;\,&space;\frac{&space;cos\left&space;(\lambda-\lambda_i&space;\right)-&space;1}{\sigma&space;_{\lambda&space;}^{2}}&space;\right)&space;&plus;&space;\nu$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$f_{ij}&space;(\theta,\lambda)&space;=&space;KCexp&space;\left&space;(&space;\frac{&space;cos\left&space;(\theta-\theta_i&space;\right)-&space;1}{\sigma&space;_{\theta&space;}^{2}}\:&space;&plus;&space;\,&space;\frac{&space;cos\left&space;(\lambda-\lambda_i&space;\right)-&space;1}{\sigma&space;_{\lambda&space;}^{2}}&space;\right)&space;&plus;&space;\nu$$" title="$$f_{ij} (\theta,\lambda) = KCexp \left ( \frac{ cos\left (\theta-\theta_i \right)- 1}{\sigma _{\theta }^{2}}\: + \, \frac{ cos\left (\lambda-\lambda_i \right)- 1}{\sigma _{\lambda }^{2}} \right) + \nu$$" /></a> 
 
où K, <a href="https://www.codecogs.com/eqnedit.php?latex=\sigma_{\theta}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\sigma_{\theta}" title="\sigma_{\theta}" /></a> et <a href="https://www.codecogs.com/eqnedit.php?latex=\sigma_{\lambda}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\sigma_{\lambda}" title="\sigma_{\lambda}" /></a>  sont des constantes, et les <a href="https://www.codecogs.com/eqnedit.php?latex=\theta_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta_{i}" title="\theta_{i}" /></a> et <a href="https://www.codecogs.com/eqnedit.php?latex=\lambda_{j}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\lambda_{j}" title="\lambda_{j}" /></a> sont réparties régulièrement sur une grille de taille <a href="https://www.codecogs.com/eqnedit.php?latex=$$P_{\theta}&space;\times&space;P_{\lambda}&space;:\;&space;\theta_i&space;=&space;2\pi&space;i&space;/P_{\theta}\,&space;,&space;\:&space;i&space;=&space;1,...,P_{\theta}&space;\;\,&space;et&space;\,&space;\;&space;\lambda_j&space;=&space;2\pi&space;j&space;/P_{\lambda}\,&space;,&space;\:&space;j&space;=&space;1,...,P_{\lambda}&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$P_{\theta}&space;\times&space;P_{\lambda}&space;:\;&space;\theta_i&space;=&space;2\pi&space;i&space;/P_{\theta}\,&space;,&space;\:&space;i&space;=&space;1,...,P_{\theta}&space;\;\,&space;et&space;\,&space;\;&space;\lambda_j&space;=&space;2\pi&space;j&space;/P_{\lambda}\,&space;,&space;\:&space;j&space;=&space;1,...,P_{\lambda}&space;$$" title="$$P_{\theta} \times P_{\lambda} :\; \theta_i = 2\pi i /P_{\theta}\, , \: i = 1,...,P_{\theta} \;\, et \, \; \lambda_j = 2\pi j /P_{\lambda}\, , \: j = 1,...,P_{\lambda} $$" /></a>. Le nombre de neurones dans le réseau est donc égal à <a href="https://www.codecogs.com/eqnedit.php?latex=$$P_{\theta}&space;\times&space;P_{\lambda}." target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$P_{\theta}&space;\times&space;P_{\lambda}." title="$$P_{\theta} \times P_{\lambda}." /></a>. Notons que la fréquence spatiale est traitée comme une variable périodique afin d’éviter les effets de bord. Le choix de fonctions circulaires normales plutôt que de gaussiennes s’explique par le fait que cette fonction est périodique.

Nous avons créé deux listes theta_grid et lambda_grid contenant les <a href="https://www.codecogs.com/eqnedit.php?latex=\theta_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta_{i}" title="\theta_{i}" /></a>  et les <a href="https://www.codecogs.com/eqnedit.php?latex=$$\lambda_j$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\lambda_j$$" title="$$\lambda_j$$" /></a>, respectivement, et une fonction prenant en arguments l’orientation θ et la fréquence spatiale λ du stimulus présenté, et retournant la matrice des <a href="https://www.codecogs.com/eqnedit.php?latex=f_{ij}(\theta,\lambda)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?f_{ij}(\theta,\lambda)" title="f_{ij}(\theta,\lambda)" /></a> :

```
theta_grid = [2*math.pi*i/P_th for i in range(1,P_th+1)]
lambda_grid = [2*math.pi*i/P_lamb for i in range(1,P_lamb+1)]

def input_tuning_curve(theta_p, lambda_p) : 
    F=np.zeros((P_th,P_lamb))
    C1 = list(map(lambda x: (math.cos(theta_p - x) -1)/sigma_th**2, theta_grid))
    C2 = list(map(lambda x: (math.cos(lambda_p - x) -1)/sigma_lamb**2, lambda_grid))
    for i in range(P_th):
        for j in range(P_lamb):    
            F[i][j] = K*C*math.exp(C1[i] + C2[j]) + nu     
    return F
```
Le bruit est choisi de sorte à suivre une distribution gaussienne de moyenne nulle et de variance égale à l’activité moyenne du neurone : <a href="https://www.codecogs.com/eqnedit.php?latex=$$\sigma_{ij}^{2}&space;=&space;f_{ij}(\theta,\lambda)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\sigma_{ij}^{2}&space;=&space;f_{ij}(\theta,\lambda)$$" title="$$\sigma_{ij}^{2} = f_{ij}(\theta,\lambda)$$" /></a> , qui est une approximation raisonnable du bruit mesuré dans le cortex – on parle alors de bruit « proportionnel »(<i>proportional noise</i>). 
Nous avons créé une fonction qui prend en argument la matrice d’activité moyenne en entrée correspondante (matrice des fij ), et qui retourne la matrice d'activité totale en entrée du réseau :
 
``` 
 def input_activity(ITC, theta_p, lambda_p) :
    A = np.zeros((P_th,P_lamb))
    for i in range(P_th):
        for j in range(P_lamb):
            variance = ITC[i][j]
            bruit = random.gauss(0,variance)
            A[i][j] = ITC[i][j] + bruit
    return A
```
L’activité à l’entrée du réseau ainsi calculée, une fois tracée dans un espace à trois dimensions en fonction de l’orientation préférentielle et de la fréquence spatiale préférentielle des neurones du réseau, se présente sous la forme d’un pic fortement bruité (noisy hill).

### Dynamique du réseau <a name="#dynamique"></a>

 A chaque itération, l’activité des neurones du réseau est mise à jour suivant les équations non-linéaires d’évolution suivantes : 

<a href="https://www.codecogs.com/eqnedit.php?latex=$$u_{ij}\left&space;(&space;t&plus;1&space;\right&space;)&space;=&space;\sum_{kl}\,&space;w_{ij,k\,&space;l}\:&space;o_{k\,&space;l}\left&space;(&space;t&space;\right&space;)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$u_{ij}\left&space;(&space;t&plus;1&space;\right&space;)&space;=&space;\sum_{kl}\,&space;w_{ij,k\,&space;l}\:&space;o_{k\,&space;l}\left&space;(&space;t&space;\right&space;)$$" title="$$u_{ij}\left ( t+1 \right ) = \sum_{kl}\, w_{ij,k\, l}\: o_{k\, l}\left ( t \right )$$" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=$$o_{ij}\left&space;(&space;t&plus;1&space;\right&space;)&space;=&space;\frac{u_{ij}\left&space;(&space;t&plus;1&space;\right&space;)^{2}}{S&plus;\mu\sum_{k\,&space;l}u_{k\,&space;l}\left&space;(&space;t&plus;1&space;\right&space;)^{2}}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$o_{ij}\left&space;(&space;t&plus;1&space;\right&space;)&space;=&space;\frac{u_{ij}\left&space;(&space;t&plus;1&space;\right&space;)^{2}}{S&plus;\mu\sum_{k\,&space;l}u_{k\,&space;l}\left&space;(&space;t&plus;1&space;\right&space;)^{2}}$$" title="$$o_{ij}\left ( t+1 \right ) = \frac{u_{ij}\left ( t+1 \right )^{2}}{S+\mu\sum_{k\, l}u_{k\, l}\left ( t+1 \right )^{2}}$$" /></a>

<img src="https://latex.codecogs.com/gif.latex?$$\left&space;\{&space;w_{i&space;j,k\,&space;l}&space;\right&space;\}$$" title="$$\left \{ w_{i j,k\, l} \right \}$$" />
où les <a href="https://www.codecogs.com/eqnedit.php?latex=$$\left&space;\{&space;w_{i&space;j,k\,&space;l}&space;\right&space;\}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\left&space;\{&space;w_{i&space;j,k\,&space;l}&space;\right&space;\}$$" title="$$\left \{ w_{i j,k\, l} \right \}$$" /></a> sont les poids de filtrage (<i>filtering weights</i>)(qui permettent de regrouper l’activité de neurones ayant des orientations et des fréquences spatiales préférentielles similaires), <a href="https://www.codecogs.com/eqnedit.php?latex=$$o_{ij}\left&space;(&space;t&space;\right&space;)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$o_{ij}\left&space;(&space;t&space;\right&space;)$$" title="$$o_{ij}\left ( t \right )$$" /></a> l’activité du neurone ij à l’instant t, S une constante, et μ est appelé le poids d’inhibition divisive (<i>divisive inhibition weight</i>). Pour les conditions initiales, l’activité du réseau à l’instant initial <a href="https://www.codecogs.com/eqnedit.php?latex=$$o_{ij}\left&space;(&space;t&space;=&space;0\right&space;)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$o_{ij}\left&space;(&space;t&space;=&space;0\right&space;)$$" title="$$o_{ij}\left ( t = 0\right )$$" /></a> est prise égale à l’activité fournie à l’entrée du réseau par la couche corticale précédente :  <a href="https://www.codecogs.com/eqnedit.php?latex=$$o_{ij}\left&space;(&space;0\right&space;)&space;=&space;a_{ij}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$o_{ij}\left&space;(&space;0\right&space;)&space;=&space;a_{ij}$$" title="$$o_{ij}\left ( 0\right ) = a_{ij}$$" /></a>, déterminée selon la méthodologie explicitée plus haut. Les poids de filtrage implémentent un filtre gaussien bidimensionnel : 

<a href="https://www.codecogs.com/eqnedit.php?latex=$$w_{ij,k\,&space;l}&space;=&space;w_{i-k,j-l}&space;=&space;K_w&space;\,&space;exp\left&space;(&space;\frac{cos\left&space;[&space;\frac{2\pi\left&space;(&space;i-k&space;\right&space;)}{P_{\theta}}&space;\right&space;]-1}{\delta_{w\theta}^{2}}&space;&plus;&space;\frac{cos\left&space;[&space;\frac{2\pi\left&space;(&space;j-l&space;\right&space;)}{P_{\lambda}}&space;\right&space;]-1}{\delta_{w\lambda}^{2}}&space;\right&space;)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$w_{ij,k\,&space;l}&space;=&space;w_{i-k,j-l}&space;=&space;K_w&space;\,&space;exp\left&space;(&space;\frac{cos\left&space;[&space;\frac{2\pi\left&space;(&space;i-k&space;\right&space;)}{P_{\theta}}&space;\right&space;]-1}{\delta_{w\theta}^{2}}&space;&plus;&space;\frac{cos\left&space;[&space;\frac{2\pi\left&space;(&space;j-l&space;\right&space;)}{P_{\lambda}}&space;\right&space;]-1}{\delta_{w\lambda}^{2}}&space;\right&space;)$$" title="$$w_{ij,k\, l} = w_{i-k,j-l} = K_w \, exp\left ( \frac{cos\left [ \frac{2\pi\left ( i-k \right )}{P_{\theta}} \right ]-1}{\delta_{w\theta}^{2}} + \frac{cos\left [ \frac{2\pi\left ( j-l \right )}{P_{\lambda}} \right ]-1}{\delta_{w\lambda}^{2}} \right )$$" /></a>


où <a href="https://www.codecogs.com/eqnedit.php?latex=$$K_w$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$K_w$$" title="$$K_w$$" /></a> est une constante, et <a href="https://www.codecogs.com/eqnedit.php?latex=$$\delta_{w\theta}^{2}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\delta_{w\theta}^{2}$$" title="$$\delta_{w\theta}^{2}$$" /></a> et <a href="https://www.codecogs.com/eqnedit.php?latex=$$\delta_{w\lambda}^{2}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\delta_{w\lambda}^{2}$$" title="$$\delta_{w\lambda}^{2}$$" /></a> contrôlent la largeur des fonctions correspondant aux poids de filtrage.

### Poids de filtrage <a name="#poids-de-filtrage"></a>

Nous avons créé une fonction retournant les poids de filtrage <a href="https://www.codecogs.com/eqnedit.php?latex=$$w_{ij,&space;k\,&space;l}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$w_{ij,&space;k\,&space;l}$$" title="$$w_{ij, k\, l}$$" /></a> sous forme de matrice de taille <a href="https://www.codecogs.com/eqnedit.php?latex=$$P_{\theta}&space;\times&space;P_{\lambda}." target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$P_{\theta}&space;\times&space;P_{\lambda}." title="$$P_{\theta} \times P_{\lambda}." /></a> dont le coefficient à la mième ligne, nième colonne est égal à <a href="https://www.codecogs.com/eqnedit.php?latex=$$w_{i-k,&space;j-l}&space;=&space;w_{m,n}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$w_{i-k,&space;j-l}&space;=&space;w_{m,n}$$" title="$$w_{i-k, j-l} = w_{m,n}$$" /></a>,	le poids de filtrage pour i-k=m et j-l=n.

```
def filtering_weights() :
    W=np.zeros((P_th,P_lamb))
    list_m = [m for m in range(P_th)]
    C = list(map(lambda x: (math.cos(x) - 1)/delta_wth**2, theta_grid))
    for m in range(P_th):
        for n in range(P_lamb): 
            W[m][n] = K_w * math.exp(C[m] + C[n])
    return W
```

### Activité en sortie du réseau <a name="#activite-en-sortie-du-reseau"></a>

Une fois les conditions initiales choisies, l’itération des deux équations d’évolution mettant à jour la dynamique du réseau entraîne la relaxation de l’activité corticale vers un état stable dans lequel elle prend la forme d’un pic lisse. Il faut seulement veiller à choisir un contraste suffisamment élevé. En pratique, on constate que 2-3 itérations suffisent pour faire converger la dynamique du réseau vers son état asymptotique. Nous avons écrit une fonction prenant en argument l’activité à l’entrée du réseau, la matrice des poids de filtrage et le nombre d’itérations des équations d’évolution, i.e. le nombre de fois que l’activité du réseau est mise à jour, et qui retourne l’activité en sortie du réseau mise à jour.
```
#Cette fonction retourne l'activité du réseau au bout de n_iterations mises à jour  

def output(input_activity, W, n_iterations) :
#Condition initiale 
    O = input_activity.copy()
    U = np.zeros((P_th,P_lamb))
#Iterations 
    for t in range(1, n_iterations +1):
        sum_u = 0.0
        for i in range(P_th) :
            for j in range(P_lamb) :   
                for k in range(P_th) :
                    for l in range(P_lamb) :
                        U[i][j] += O[k][l]*W[i-k][j-l]
                sum_u += U[i][j]**2
        O[:,:] = (U[:,:]**2)/(S + mu * sum_u )
    return O
```

### Estimations de l’orientation et de la fréquence spatiale encodées par le réseau  <a name="#estimations"></a>

La position de ce pic d’activité dépend de l’orientation θ et de la fréquence spatiale λ du stimulus présenté, et permet donc d’obtenir, pour chaque essai, des estimations de ces deux quantités, notées <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\theta}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\theta}$$" title="$$\hat{\theta}$$" /></a> et <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\lambda}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\lambda}$$" title="$$\hat{\lambda}$$" /></a>. Ainsi, à partir de l’activité en sortie du réseau, <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\theta}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\theta}$$" title="$$\hat{\theta}$$" /></a> et <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\lambda}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\lambda}$$" title="$$\hat{\lambda}$$" /></a>peuvent être déterminées en utilisant un estimateur complexe, équivalent à un estimateur de vecteur de population : 

<a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\theta}&space;=&space;phase\left&space;(&space;\sum_{kj}o_{kj}&space;\,&space;e^{i\theta_j}&space;\right&space;)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\theta}&space;=&space;phase\left&space;(&space;\sum_{kj}o_{kj}&space;\,&space;e^{i\theta_j}&space;\right&space;)$$" title="$$\hat{\theta} = phase\left ( \sum_{kj}o_{kj} \, e^{i\theta_j} \right )$$" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\lambda}&space;=&space;phase\left&space;(&space;\sum_{kj}o_{kj}&space;\,&space;e^{i\lambda_j}&space;\right&space;)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\lambda}&space;=&space;phase\left&space;(&space;\sum_{kj}o_{kj}&space;\,&space;e^{i\lambda_j}&space;\right&space;)$$" title="$$\hat{\lambda} = phase\left ( \sum_{kj}o_{kj} \, e^{i\lambda_j} \right )$$" /></a>

Nous avons donc écrit deux fonctions, theta_estimator(output) et lambda_estimator(output), qui prennent en argument l’activité en sortie du réseau et retournent respectivement les estimations <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\theta}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\theta}$$" title="$$\hat{\theta}$$" /></a> et <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\lambda}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\lambda}$$" title="$$\hat{\lambda}$$" /></a>: 

```
#Cette fonction retourne l'estimation de l'orientation du stimulus sur
# un essai calculée à partir de l'activité du réseau

def theta_estimator(output) :
    Z = 0.0 + 0.0*1j
    C = list(map(lambda x: math.cos(x), theta_grid))
    S = list(map(lambda x: math.sin(x), theta_grid))
    C = np.array(C)
    S = np.array(S)
    for k in range(P_th):
        for j in range(P_lamb):
            Z += output[k][j]*C[j] + output[k][j]*S[j]*1j
    phase = cmath.phase(Z)
    if phase < 0.0:
        phase = phase + 2*math.pi # pour que l'angle calculé soit dans l'intervalle [0;2π] 
    return phase

#Cette fonction retourne l'estimation de la fréquence spatiale du stimulus sur
# un essai calculée à partir de l'activité du réseau

def lambda_estimator(output) :
    Z = 0.0 + 0.0*1j
    C = list(map(lambda x: math.cos(x), lambda_grid))
    S = list(map(lambda x: math.sin(x), lambda_grid))
    C = np.array(C)
    S = np.array(S)
    for k in range(P_th):
        for j in range(P_lamb):
            Z += output[k][j]*C[j] + output[k][j]*S[j]*1j
    phase = cmath.phase(Z)
    if phase < 0.0:
        phase = phase + 2*math.pi # pour que la fréquence calculée soit dans l'intervalle [0;2π] 
    return phase
```

### Moyenne et variance des estimations <a name="#moyenne-et-variance-des-estimations"></a>

Il est alors possible de calculer la moyenne et la variance de l’estimation sur un grand nombre d’essais, ce qui permet in fine d’en évaluer sa qualité. Dans notre réseau, nous avons choisi des courbes d’accord, un bruit et des poids de filtrage qui restent invariants lorsque l’on interchange θ et λ. Par conséquent, les variances de <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\theta}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\theta}$$" title="$$\hat{\theta}$$" /></a> et <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\lambda}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\lambda}$$" title="$$\hat{\lambda}$$" /></a> sont identiques, et pour simplifier et gagner en temps de calcul, on ne s’intéressera qu’à la variance sur  <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\theta}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\theta}$$" title="$$\hat{\theta}$$" /></a> dans un premier temps. 

Afin de déterminer la moyenne et la variance de <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\theta}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\theta}$$" title="$$\hat{\theta}$$" /></a> pour un jeu donné de paramètres du réseau, nous avons écrit une fonction stats_orientation_net(theta_p, lambda_p, ITC, W, n_iter) qui prend en argument l’orientation θ et de la fréquence spatiale λ du stimulus présenté, la matrice des courbes d’accord des neurones 〖 f〗_ij (θ,λ)  (calculée séparément), la matrice des poids de filtrage et le nombre de mises à jour de l’activité du réseau, et retourne la moyenne et la variance calculées sur n_trials = 1000 essais différents. A chaque essai, les valeurs de l’orientation θ et de la fréquence spatiale λ du stimulus en entrée du réseau étant donc fixées, cette fonction génère une nouvelle matrice d’activité totale, bruitée, en entrée du réseau, met à jour un nombre n_iter de fois l’activité du réseau jusqu’à relaxation et apparition d’un pic lisse, puis calcule les estimations<a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\theta}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\theta}$$" title="$$\hat{\theta}$$" /></a> et <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\lambda}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\lambda}$$" title="$$\hat{\lambda}$$" /></a>. 

Enfin, nous avons testé notre programme en considérant un réseau de 20 × 20 neurones (P_θ= P_λ=20) avec les valeurs des paramètres suivantes : <a href="https://www.codecogs.com/eqnedit.php?latex=$$K&space;=&space;74,&space;\;&space;C&space;=&space;1,&space;\;&space;\nu&space;=&space;3.7,&space;\;&space;\sigma_{\theta}&space;=&space;\sigma_{\lambda}&space;=&space;\delta_{w\theta}&space;=&space;\delta_{w\lambda}&space;=&space;0.38,&space;\;&space;\mu&space;=&space;0.002,&space;\;&space;K_w&space;=&space;1&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$K&space;=&space;74,&space;\;&space;C&space;=&space;1,&space;\;&space;\nu&space;=&space;3.7,&space;\;&space;\sigma_{\theta}&space;=&space;\sigma_{\lambda}&space;=&space;\delta_{w\theta}&space;=&space;\delta_{w\lambda}&space;=&space;0.38,&space;\;&space;\mu&space;=&space;0.002,&space;\;&space;K_w&space;=&space;1&space;$$" title="$$K = 74, \; C = 1, \; \nu = 3.7, \; \sigma_{\theta} = \sigma_{\lambda} = \delta_{w\theta} = \delta_{w\lambda} = 0.38, \; \mu = 0.002, \; K_w = 1 $$" /></a>.  Nous choisissons arbitrairement une orientation du stimulus <a href="https://www.codecogs.com/eqnedit.php?latex=$$\theta&space;=&space;4\pi/3&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\theta&space;=&space;4\pi/3&space;$$" title="$$\theta = 4\pi/3 $$" /></a> et une fréquence spatiale λ = 3.

Pour cela, nous calculons tout d’abord la matrice des poids de filtrage et la matrice d'activité moyenne en entrée du réseau (matrice des <a href="https://www.codecogs.com/eqnedit.php?latex=f_{ij}(\theta,\lambda)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?f_{ij}(\theta,\lambda)" title="f_{ij}(\theta,\lambda)" /></a> : 

```
#Calcul de la matrice des poids de filtrage 
W = filtering_weights()

#Calcul de la matrice d'activité moyenne en entrée du réseau     
F = input_tuning_curve(theta_p, lambda_p) 

```
Dans un premier temps, nous réalisons un essai unique afin de visualiser les activités en entrée et en sortie du réseau au bout de 3 itérations des équations d’évolution. Pour cela, nous bruitons d’abord l’activité moyenne en entrée du réseau :


```
#Initialisation de la matrice d'activité totale en entrée du réseau    
A = input_activity(F)

######## Visualisation de l'activité du réseau immédiatement après initialisation ##############
initial_activity = output(A, W, 0)

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
#On convertit la matrice de l'activité initiale du réseau en array à deux dimensions
Z0=[]
for i in range(P_th) :
    for j in range(P_lamb) :                
       Z0.append(initial_activity[i][j])
Z0 = np.array(Z0)
#On convertit la liste lambda_grid en array 
Y = []
for i in range(P_th):
    for j in range(P_lamb):
        Y.append(lambda_grid[j])
Y = np.array(Y)  
#On convertit la liste theta_grid en array     
X = []
for i in range(P_th):
    for j in range(P_lamb):
        X.append(180*theta_grid[i]/math.pi) #on convertit les orientations préférentielles theta_grid[i] en degrés
X = np.array(X)
    
xs = X.reshape((-1, P_th))
ys = Y.reshape((-1, P_th))
zs = Z0.reshape((-1, P_th))  
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')  
ax.plot_surface(xs,ys,zs, rstride=1, cstride=1)
plt.show()
```

Puis nous faisons tourner la dynamique du réseau en faisant appel à la fonction output (A, W, n_iterations) qui calcule l'activité en sortie du réseau au bout de n_iterations = 3 mises à jour.
```
######## Visualisation de l'activité du réseau mise à jour après 3 iterations #########

#Calcul de l'activité en sortie du réseau au bout de n_iterations = 3 mises à jour
output_activity = output(A, W, n_iterations)

#On convertit la matrice de l'activité en sortie du réseau en array à deux dimensions
Z=[]
for i in range(P_th) :
    for j in range(P_lamb) :                
        Z.append(output_activity[i][j])
Z = np.array(Z)

zs = Z.reshape((-1, P_th))  
fig1 = plt.figure()
ax = fig1.add_subplot(111, projection='3d')  
ax.plot_surface(xs,ys,zs, rstride=1, cstride=1)
plt.xlabel('theta')
plt.ylabel('lambda')
#plt.zlabel('Activity')
plt.show()
```
Dans un second temps, on s’intéresse à la performance de ce réseau dans l’estimation de l’orientation et de la fréquence spatiale du stimulus. Pour cela, on fait appel à la fonction stats_net(theta_p, lambda_p, F, W, n_iterations) qui retourne directement les moyennes et variances
respectives des estimations <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\theta}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\theta}$$" title="$$\hat{\theta}$$" /></a> et <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\lambda}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\lambda}$$" title="$$\hat{\lambda}$$" /></a>,  sur n_iterations = 1000 essais. On constate que les estimations sont non-biaisées, i.e. les valeurs moyennes de <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\theta}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\theta}$$" title="$$\hat{\theta}$$" /></a> et <a href="https://www.codecogs.com/eqnedit.php?latex=$$\hat{\lambda}$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\hat{\lambda}$$" title="$$\hat{\lambda}$$" /></a> convergent vers les valeurs véritables de θ et λ. De plus, les valeurs des variances demeurent très faibles également. 

Conclusion 

Nous avons ainsi simulé une classe particulière de réseaux récurrents de neurones, que l’on retrouve dans tout le cortex, caractérisés par des courbes d’accord larges et dont la fonction d’activation est non linéaire et comprend une normalisation divisive. Nous avons montré que ce type de réseaux permet d’extraire des variables encodées dans l’activité bruitée d’une population de neurones. Il resterait à implémenter un estimateur de maximum de vraisemblance et à comparer sa variance avec celle de l’estimateur représenté par notre réseau afin de prouver qu’il s’agit bien d’un estimateur idéal. Ceci montrerait que les aires corticales peuvent être ajustées de sorte à réaliser une estimation au maximum de vraisemblance, ce qui suggèrerait que la capacité à traiter et décoder de manière optimale l’information contenue dans une entrée bruitée est une propriété générale du cortex.
























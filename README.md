# PCBS-IdealObserver

Programmation d’un réseau récurrent de neurones capable d’implémenter un estimateur de type ‘observateur idéal’.
 
Introduction

De nombreux neurones du cortex visuel primaire sont sensibles à l’orientation du stimulus présenté. La courbe représentant l’activité de réponse d’un neurone en fonction de l’orientation présentée, appelée courbe d’accord (tuning curve), est typiquement en forme de cloche, la position du maximum correspondant à l’orientation préférentielle du neurone. Elle permet de prédire la réponse moyenne d’un neurone à une orientation donnée. Cependant, la réponse individuelle pour une même orientation varie d’un essai à l’autre en raison du bruit neuronal. Cette variabilité est particulièrement visible lorsque l’on trace l’activité d’une population entière de neurones en réponse à une grille d’une orientation θ donnée. Pour un essai donné, en traçant l’activité de chaque neurone de la population en fonction de son orientation préférentielle, on obtient un pic très bruité. Un autre essai avec la même orientation du stimulus conduirait à une courbe similaire, mais avec une réponse légèrement différente pour chaque neurone, et donc une position du pic central différente.


Il s’agit alors pour le cerveau d’estimer la valeur de l’orientation encodée dans cette activité bruitée d’une population de neurones. Une des méthodes classiquement proposées pour effectuer cette tâche s’appuie sur le vecteur population de neurones. Il s’agit d’un estimateur facile à implémenter et sans biais, c’est-à-dire que l’estimation ainsi obtenue (qui varie d’un essai à l’autre en même temps que l’activité de la population varie), est égale en moyenne (sur les essais) à l’orientation présentée. Toutefois, cet estimateur n’est pas optimal car sa variance sur l’estimation est bien supérieure à la borne inférieure sur la variance pour un estimateur sans biais (borne de Cramér-Rao). Or idéalement, l’estimateur devrait avoir une variance minimale, c’est-à-dire que l’estimation devrait varier le moins possible d’un essai à l’autre, lorsque l’orientation est fixée. Cette borne inférieure est dictée par la structure du bruit neuronal ; elle est atteinte pour l’estimateur dit du maximum de vraisemblance : on parle d’observateur idéal, car il optimise l’estimation compte tenu du bruit.
 L’estimation de variables encodées dans l’activité de populations de neurones ne se limite pas toutefois à l’orientation, et d’autres variables sensorielles ou motricielles peuvent être encodées. Dans ce projet nous considèrerons deux variables : l’orientation et la fréquence spatiale d’une grille. 

L’objectif de ce projet est de simuler un réseau de neurones récurrents capable d’implémenter un estimateur sans biais de ces deux variables, et dont la variance est égale à (ou très proche de, suivant le type de bruit) la variance minimale atteinte par l’estimateur au maximum de vraisemblance. On cherche donc à implémenter un estimateur de type observateur idéal. Pour cela, le principe consiste à utiliser un réseau de neurones récurrents dont la fonction d’activation comprend une normalisation divisive (divisive normalization), dont l’expression sera explicitée plus loin, reproduisant les fonctions d’activations observées pour des neurones du cortex visuel primaire. Il a en effet été prouvé (Denève et al., 1999) que le réseau de neurones qui en résulte permet d’implémenter un estimateur dont la variance est égale à la variance minimale atteinte par le maximum de vraisemblance dans le cas d’un bruit neuronal indépendant du taux de décharge, et très proche de la valeur minimale dans le cas plus réaliste, considéré ici, d’un bruit poissonnien. 

Le réseau que nous considérons modélise une colonne corticale constituée d’une seule couche de neurones (unités ij) dont les champs récepteurs sont identiques, mais qui diffèrent par leurs orientations et fréquences spatiales préférentielles. Chaque neurone est repéré par deux indices, et le neurone ij est caractérisé par son orientation préférentielle <a href="https://www.codecogs.com/eqnedit.php?latex=\theta_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta_{i}" title="\theta_{i}" /></a>	et sa fréquence spatiales <a href="https://www.codecogs.com/eqnedit.php?latex=\lambda_{j}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\lambda_{j}" title="\lambda_{j}" /></a>. Le réseau reçoit une activité en entrée provenant d’une couche précédente, qui représente soit une autre couche corticale soit le noyau géniculé latéral. (Nous ne modélisons donc pas explicitement cette couche, et nous concentrons sur l’entrée qu’elle fournit à chaque 
neurone de notre réseau).  On désignera l’activité du réseau qui en résulte sous le terme d’activité en sortie (output activity).

L’entrée fournie au réseau dépend de l’orientation θ et de la fréquence spatiale λ du stimulus présenté, qui sont encodées dans la couche précédente. Pour des valeurs de θ et λ données, l’activité totale en entrée du neurone ij, <a href="https://www.codecogs.com/eqnedit.php?latex=a_{ij}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?a_{ij}" title="a_{ij}" /></a>, est la somme de deux termes : le premier correspond à l’entrée moyenne  <a href="https://www.codecogs.com/eqnedit.php?latex=f_{ij}(\theta,\lambda)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?f_{ij}(\theta,\lambda)" title="f_{ij}(\theta,\lambda)" /></a>, le second <a href="https://www.codecogs.com/eqnedit.php?latex=\xi_{ij}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\xi_{ij}" title="\xi_{ij}" /></a> à un terme de bruit autour de cette activité moyenne en entrée :

<a href="https://www.codecogs.com/eqnedit.php?latex=a_{ij}=&space;f_{ij}&space;(\theta&space;,\lambda&space;)&plus;&space;\xi_{ij}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?a_{ij}=&space;f_{ij}&space;(\theta&space;,\lambda&space;)&plus;&space;\xi_{ij}" title="a_{ij}= f_{ij} (\theta ,\lambda )+ \xi_{ij}" /></a>

L’activité moyenne en entrée du neurone ij pour un stimulus d’orientation θ, de longueur d’onde spatiale λ et de contraste C,<a href="https://www.codecogs.com/eqnedit.php?latex=$$P_{\theta}&space;\times&space;P_{\lambda}." target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$P_{\theta}&space;\times&space;P_{\lambda}." title="$$P_{\theta} \times P_{\lambda}." /></a>, est choisie de sorte à reproduire des courbes d’accord (tuning curves) physiologiquement réalistes – des profils en forme de cloche dont l’amplitude est proportionnelle au contraste. Pour cela, on choisit de prendre des fonctions circulaires normales avec un terme d’activité spontanée ν en plus :

<a href="https://www.codecogs.com/eqnedit.php?latex=$$f_{ij}&space;(\theta,\lambda)&space;=&space;KCexp&space;\left&space;(&space;\frac{&space;cos\left&space;(\theta-\theta_i&space;\right)-&space;1}{\sigma&space;_{\theta&space;}^{2}}\:&space;&plus;&space;\,&space;\frac{&space;cos\left&space;(\lambda-\lambda_i&space;\right)-&space;1}{\sigma&space;_{\lambda&space;}^{2}}&space;\right)&space;&plus;&space;\nu$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$f_{ij}&space;(\theta,\lambda)&space;=&space;KCexp&space;\left&space;(&space;\frac{&space;cos\left&space;(\theta-\theta_i&space;\right)-&space;1}{\sigma&space;_{\theta&space;}^{2}}\:&space;&plus;&space;\,&space;\frac{&space;cos\left&space;(\lambda-\lambda_i&space;\right)-&space;1}{\sigma&space;_{\lambda&space;}^{2}}&space;\right)&space;&plus;&space;\nu$$" title="$$f_{ij} (\theta,\lambda) = KCexp \left ( \frac{ cos\left (\theta-\theta_i \right)- 1}{\sigma _{\theta }^{2}}\: + \, \frac{ cos\left (\lambda-\lambda_i \right)- 1}{\sigma _{\lambda }^{2}} \right) + \nu$$" /></a> 
 
où K, <a href="https://www.codecogs.com/eqnedit.php?latex=\sigma_{\theta}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\sigma_{\theta}" title="\sigma_{\theta}" /></a> et <a href="https://www.codecogs.com/eqnedit.php?latex=\sigma_{\lambda}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\sigma_{\lambda}" title="\sigma_{\lambda}" /></a>  sont des constantes, et les <a href="https://www.codecogs.com/eqnedit.php?latex=\theta_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta_{i}" title="\theta_{i}" /></a> et <a href="https://www.codecogs.com/eqnedit.php?latex=\lambda_{j}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\lambda_{j}" title="\lambda_{j}" /></a> sont réparties régulièrement sur une grille de taille <a href="https://www.codecogs.com/eqnedit.php?latex=$$P_{\theta}&space;\times&space;P_{\lambda}&space;:\;&space;\theta_i&space;=&space;2\pi&space;i&space;/P_{\theta}\,&space;,&space;\:&space;i&space;=&space;1,...,P_{\theta}&space;\;\,&space;et&space;\,&space;\;&space;\lambda_j&space;=&space;2\pi&space;j&space;/P_{\lambda}\,&space;,&space;\:&space;j&space;=&space;1,...,P_{\lambda}&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$P_{\theta}&space;\times&space;P_{\lambda}&space;:\;&space;\theta_i&space;=&space;2\pi&space;i&space;/P_{\theta}\,&space;,&space;\:&space;i&space;=&space;1,...,P_{\theta}&space;\;\,&space;et&space;\,&space;\;&space;\lambda_j&space;=&space;2\pi&space;j&space;/P_{\lambda}\,&space;,&space;\:&space;j&space;=&space;1,...,P_{\lambda}&space;$$" title="$$P_{\theta} \times P_{\lambda} :\; \theta_i = 2\pi i /P_{\theta}\, , \: i = 1,...,P_{\theta} \;\, et \, \; \lambda_j = 2\pi j /P_{\lambda}\, , \: j = 1,...,P_{\lambda} $$" /></a>. Le nombre de neurones dans le réseau est donc égal à <a href="https://www.codecogs.com/eqnedit.php?latex=$$P_{\theta}&space;\times&space;P_{\lambda}." target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$P_{\theta}&space;\times&space;P_{\lambda}." title="$$P_{\theta} \times P_{\lambda}." /></a>. Notons que la fréquence spatiale est traitée comme une variable périodique afin d’éviter les effets de bord. Le choix de fonctions circulaires normales plutôt que de gaussiennes s’explique par le fait que cette fonction est périodique.

Nous avons créé deux listes theta_grid et lambda_grid contenant les <a href="https://www.codecogs.com/eqnedit.php?latex=\theta_{i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta_{i}" title="\theta_{i}" /></a>  et les λ_j, respectivement, et une fonction prenant en arguments l’orientation θ et la fréquence spatiale λ du stimulus présenté, et retournant la matrice des <a href="https://www.codecogs.com/eqnedit.php?latex=f_{ij}(\theta,\lambda)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?f_{ij}(\theta,\lambda)" title="f_{ij}(\theta,\lambda)" /></a> :

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
Le bruit est choisi de sorte à suivre une distribution gaussienne de moyenne nulle et de variance égale à l’activité moyenne du neurone :<a href="https://www.codecogs.com/eqnedit.php?latex=$$\sigma_{ij}^{2}&space;=&space;f_{ij}(\theta,\lambda)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$\sigma_{ij}^{2}&space;=&space;f_{ij}(\theta,\lambda)$$" title="$$\sigma_{ij}^{2} = f_{ij}(\theta,\lambda)$$" /></a>, qui est une approximation raisonnable du bruit mesuré dans le cortex – on parle alors de bruit « proportionnel »(<i>proportional noise<i>). 
Nous avons créé une fonction qui prend en argument la matrice d’activité moyenne en entrée correspondante (matrice des fij ), et qui retourne la matrice d'activité totale en entrée du réseau :


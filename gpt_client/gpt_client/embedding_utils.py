import os
from langchain.text_splitter import CharacterTextSplitter
from langchain.document_loaders import TextLoader

EMBEDDING_MODEL = 1  # O for OpenAIEmbeddings and 1 for HuggingFaceEmbeddings
if EMBEDDING_MODEL == 0:
    from langchain.embeddings.openai import OpenAIEmbeddings
elif EMBEDDING_MODEL == 1:
    from langchain.embeddings import HuggingFaceEmbeddings

VECTOR_STORE = 1  # 0 for Pinecone and 1 for Chroma
if VECTOR_STORE == 0:
    import pinecone
    from langchain.vectorstores import Pinecone
elif VECTOR_STORE == 1:
    # make sure you have chromadb on your local machine `pip install chromadb`
    from langchain.vectorstores import Chroma


def load_docs():
    """
    Load documents from a text file and split them into chunks of 1000 characters.
    """
    loader = TextLoader(os.path.join(os.path.dirname(__file__), "prompts/task_settings.txt"))
    documents = loader.load()
    text_splitter = CharacterTextSplitter(separator="---", chunk_size=500, chunk_overlap=20)
    docs = text_splitter.split_documents(documents)
    return docs


def init_embedding_model(model_name = 'flax-sentence-embeddings/all_datasets_v4_MiniLM-L6'):
    """
    Initialize embeddings
    """
    if EMBEDDING_MODEL == 0:
        embeddings = OpenAIEmbeddings()  # OpenAI default embedding

    elif EMBEDDING_MODEL == 1:
        embeddings = HuggingFaceEmbeddings(
                        model_name=model_name,
                        model_kwargs={"device": "cuda"},
                    )
    return embeddings

def init_vector_store(embedding_model, is_already_indexed=True):
    """
    Initialize vector store
    """
    if VECTOR_STORE == 0:
        vector_store = init_pinecone_vector_store(embedding_model, is_already_indexed)
    elif VECTOR_STORE == 1:
        vector_store = init_chroma_vector_store(embedding_model, is_already_indexed)
    return vector_store

def init_pinecone_vector_store(embedding_model, is_already_indexed=True):
    """
    Initialize pinecone vector store
    """
    pinecone.init(api_key=os.getenv("PINECONE_API_KEY"), environment="us-west4-gcp-free")
    
    index_name = "codellama-vs"
    # Check if our index already exists. If it doesn't, we create it
    if index_name not in pinecone.list_indexes():
        # we create a new index
        # The OpenAI embedding model `text-embedding-ada-002` uses 1536 dimensions
        # The Hg embedding model 'flax-sentence-embeddings/all_datasets_v4_MiniLM-L6' uses 384 dimensions
        pinecone.create_index(
            name=index_name,
            metric='cosine',
            dimension=384  
        )

    docs = load_docs()

    # if you don't have an index, you can create one like this
    if not is_already_indexed:
        vector_store = Pinecone.from_documents(docs, embedding_model, index_name=index_name)
    else:
        # if you already have an index, you can load it like this
        vector_store = Pinecone.from_existing_index(index_name, embedding_model)
    return vector_store

def init_chroma_vector_store(embedding_model, is_already_indexed=True):
    """
    Initialize chroma vector store
    """
    docs = load_docs()

    vector_store = Chroma.from_documents(docs, embedding_model)

    return vector_store
